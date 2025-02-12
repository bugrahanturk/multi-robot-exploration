#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GetPlan.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <unordered_map>
#include <queue>
#include <vector>
#include <set>         
#include <algorithm>

class MultiRobotExplorer 
{
private:
    /**
     * @struct Cell
     * @brief Harita üzerinde bir hücrenin (x, y) konumu ve etiket bilgisini tutar.
     */
    struct Cell 
    {
        int x, y;  ///< Hücrenin grid içindeki (x,y) indeksleri
        int label; ///< CCL sırasında verilen etiket
        Cell(int x_, int y_, int label_ = 0) : x(x_), y(y_), label(label_) {}
    };

    /**
     * @struct FrontierCluster
     * @brief Frontier hücrelerinin kümelenmesiyle oluşan bir grubun bilgilerini tutar.
     */
    struct FrontierCluster 
    {
        std::vector<Cell> cells;
        geometry_msgs::Point centroid;
        double utility;
        bool assigned;
        int size;
        double information_potential;  //!< Küme etrafındaki bilinmeyen alan tahmini

        FrontierCluster() : utility(0.0), assigned(false), size(0), information_potential(0.0) {}
    };

    /**
     * @struct Robot
     * @brief Her bir TurtleBot3 robotuna ait move_base action client, plan client
     *        ve odom bilgisini tutar.
     */
    struct Robot 
    {
        std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> action_client; //!< move_base ActionClient
        std::string name;                                                                             //!< Robotun adı (tb3_0, tb3_1, vb.)
        geometry_msgs::PoseStamped current_pose;                                                      //!< Robotun harita çerçevesindeki mevcut konumu
        ros::Subscriber pose_sub;                                                                     //!< Odom aboneliği
        ros::ServiceClient plan_client;                                                               //!< navfn ile plan almak için
        bool is_available;                                                                            //!< Robotun hedef almaya müsait olup olmadığı

        Robot() : is_available(true) {}
    };
    
    /**
     * @struct PathCache
     * @brief Robot ve frontier kümeleri arasındaki önceden hesaplanmış plan uzunluklarını
     *        geçici olarak saklar, tekrar hesaplamayı önleyerek performans kazandırır.
     */
    struct PathCache 
    {
         /**
         * @struct PathKey
         * @brief Bir robotla bir kümenin (cluster) eşleşmesini benzersiz anahtar olarak tutar.
         */
        struct PathKey 
        {
            std::string robot_name;
            int cluster_id;
            
            bool operator==(const PathKey& other) const 
            {
                return (robot_name == other.robot_name && cluster_id == other.cluster_id);
            }
        };
        
         /**
         * @struct PathKeyHash
         * @brief PathKey yapısını std::unordered_map içinde kullanmak için özel hash fonksiyonu.
         */
        struct PathKeyHash 
        {
            std::size_t operator()(const PathKey& k) const 
            {
                //!< Robot adı ve küme ID'si üzerinden basit XOR hash
                return std::hash<std::string>()(k.robot_name) ^ std::hash<int>()(k.cluster_id);
            }
        };

        std::unordered_map<PathKey, double, PathKeyHash> path_lengths;
        ros::Time last_update;
        const double CACHE_DURATION = 2.0; // saniye (örn. 2 saniye geçerli)

        /**
         * @brief Önbelleğin hâlâ geçerli olup olmadığını kontrol eder.
         */
        bool isValid() const 
        {
            return (ros::Time::now() - last_update).toSec() < CACHE_DURATION;
        }

        /**
         * @brief Önbelleği temizler (tüm kayıtlar silinir).
         */
        void clear() 
        {
            path_lengths.clear();
        }
    };

    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::vector<Robot> robots_;
    std::vector<FrontierCluster> clusters_;
    nav_msgs::OccupancyGrid current_map_;
    
    PathCache path_cache_;
    
    //!< Parametreler
    const double ALPHA = 0.3;               //!< Mesafe ağırlığı
    const double MIN_UTILITY = -10;         //!< Minimum utility eşik
    const int MIN_CLUSTER_SIZE = 5;         //!< Minimum frontier kümesi boyutu
    const double MAX_PLAN_DISTANCE = 100.0; //!< metre
    const int MAX_ALTERNATIVE_TARGETS = 5;  //!< Engelli centroid durumunda denenecek hücre sayısı

public:
    MultiRobotExplorer() 
      : nh_("~"), tf_listener_(tf_buffer_)
    {
        std::vector<std::string> robot_names = {"tb3_0", "tb3_1", "tb3_2", "tb3_3"};
        
        for (const auto& name : robot_names) 
        {
            Robot robot;
            robot.name = name;
            
            //!<  move_base ActionClient oluştur
            robot.action_client.reset(
                new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(
                    name + "/move_base", true
                )
            );
            
            //!< Odom subscription
            robot.pose_sub = nh_.subscribe<nav_msgs::Odometry>(
                "/" + name + "/odom", 
                1,
                [this, &robot](const nav_msgs::Odometry::ConstPtr& msg) {
                    this->poseCallback(msg, &robot);
                }
            );
            
            //!< Plan service client
            robot.plan_client = nh_.serviceClient<nav_msgs::GetPlan>("/" + name + "/move_base/make_plan");
            
            robots_.push_back(std::move(robot));
        }

        //!< Map subscription
        map_sub_ = nh_.subscribe("/map", 1, &MultiRobotExplorer::mapCallback, this);
    }

     /**
     * @brief Robotun odom bilgisini alıp harita çerçevesine dönüştürür.
     * @param msg Odom verisi
     * @param robot İlgili robot yapısı
     */
    void poseCallback(const nav_msgs::Odometry::ConstPtr& msg, Robot* robot) 
    {   
        //!< Odom frame'inden map frame'ine dönüşüm
        try
         {
            robot->current_pose.header = msg->header;
            robot->current_pose.pose = msg->pose.pose;
            geometry_msgs::TransformStamped transform = 
                tf_buffer_.lookupTransform("map", 
                                           robot->current_pose.header.frame_id,
                                           ros::Time(0), 
                                           ros::Duration(1.0));

            tf2::doTransform(robot->current_pose, robot->current_pose, transform);
        } 
        catch (tf2::TransformException &ex) 
        {
            ROS_WARN("%s icin TF hatasi: %s", robot->name.c_str(), ex.what()); 
        }
    }

    /**
     * @brief /map callback fonksiyonu - Frontier kümelerini bulur ve atama yapar.
     * @param map Alınan OccupancyGrid
     */
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map) 
    {
        current_map_ = *map;
        findFrontierClusters();
        
        // Müsait robot var mı?
        bool any_robot_available = false;
        for (const auto& robot : robots_) 
        {
            if (robot.is_available) 
            {
                any_robot_available = true;
                break;
            }
        }
        
        if (any_robot_available) 
        {
            assignTargets();
        }
    }

    /**
     * @brief move_base doneCallback - Robot hedefe ulaştığında veya iptal/başarısız olduğunda çağrılır.
     * @param state ActionClient durum
     * @param result move_base sonucu
     * @param robot İlgili robot
     */
    void doneCallback(const actionlib::SimpleClientGoalState& state,
                     const move_base_msgs::MoveBaseResultConstPtr& result,
                     Robot* robot) 
    {
        ROS_INFO("%s Reached target. Status: %s",robot->name.c_str(), state.toString().c_str());

        robot->is_available = true;

        // Frontier'ları yeniden bul
        findFrontierClusters();
        
        // Eğer frontier varsa yeni hedef
        if (!clusters_.empty()) 
        {
            assignTargets();
        }
    }

    /**
     * @brief Hedef aktifleştiğinde (move_base) çağrılır.
     */
    void activeCallback() 
    {
        ROS_INFO("Goal is now active");
    }

    /**
     * @brief move_base'den feedback alındığında çağrılır.
     */
    void feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) 
    {
        ROS_INFO("Got feedback from move_base");
    }

        /**
     * @brief Bir (x,y) hücresinin frontier (sınır) olup olmadığını denetler.
     *        Hücre boşsa (0) ve 8-komşuda en az bir -1 (unknown) varsa frontier sayılır.
     * @param x Hücrenin x indeksi
     * @param y Hücrenin y indeksi
     * @return Frontier ise true, aksi halde false
     */
    bool isFrontierCell(int x, int y) 
    {
        int idx = y * current_map_.info.width + x;
        //!< Hücre boş mu?
        if (current_map_.data[idx] == 0) 
        {
            //!< 8-komşulukta bilinmeyen alan var mı?
            for (int dy = -1; dy <= 1; dy++) 
            {
                for (int dx = -1; dx <= 1; dx++) 
                {
                    if (dx == 0 && dy == 0)
                    {
                        continue;
                    }
                    
                    int nx = x + dx;
                    int ny = y + dy;
                    int nidx = ny * current_map_.info.width + nx;
                    
                    if (current_map_.data[nidx] == -1) 
                    {  
                        return true;
                    }
                }
            }
        }
        return false;
    }

    /**
     * @brief BFS ile frontier hücrelerini etiketleyerek kümelendirir.
     * @param x Başlangıç hücresinin x indeksi
     * @param y Başlangıç hücresinin y indeksi
     * @param label Bu kümenin etiketi
     * @param labels Etiket matrisi
     */
    void labelComponent(int x, int y, int label, std::vector<std::vector<int>>& labels) 
    {
        std::queue<Cell> q;
        q.push(Cell(x, y));
        labels[y][x] = label;

        while (!q.empty()) 
        {
            Cell current = q.front();
            q.pop();

            //!< 8-komşuluk
            for (int dy = -1; dy <= 1; dy++) 
            {
                for (int dx = -1; dx <= 1; dx++) 
                {
                    if (dx == 0 && dy == 0)
                    {
                        continue;
                    }

                    int nx = current.x + dx;
                    int ny = current.y + dy;

                    if (nx >= 0 && nx < current_map_.info.width &&
                        ny >= 0 && ny < current_map_.info.height &&
                        labels[ny][nx] == 0 && isFrontierCell(nx, ny)) 
                        {
                            labels[ny][nx] = label;
                            q.push(Cell(nx, ny));
                    }
                }
            }
        }
    }

    /**
     * @brief Haritadaki tüm frontier hücrelerini bulur, BFS ile kümeler ve minimum boyuttaki kümeleri eler.
     */
    void findFrontierClusters() 
    {
        if (current_map_.info.width == 0 || current_map_.info.height == 0) 
        {
            ROS_WARN("Map is empty or not received yet.");
            return;
        }

        //!< 2D label matrisi
        std::vector<std::vector<int>> labels(current_map_.info.height, std::vector<int>(current_map_.info.width, 0));
        int current_label = 1;
        
        //!< Frontier hücrelerini etiketle
        for (int y = 1; y < (int)current_map_.info.height - 1; y++) 
        {
            for (int x = 1; x < (int)current_map_.info.width - 1; x++) 
            {
                if (isFrontierCell(x, y))
                 {
                    if (labels[y][x] == 0) 
                    {
                        labelComponent(x, y, current_label, labels);
                        current_label++;
                    }
                }
            }
        }

        //!< cluster vektörünü oluştur
        clusters_.clear();
        clusters_.resize(current_label);

        //!< Hücreleri kümelere dağıt
        for (int y = 0; y < (int)current_map_.info.height; y++) 
        {
            for (int x = 0; x < (int)current_map_.info.width; x++) 
            {
                int lbl = labels[y][x];
                if (lbl > 0) 
                {
                    clusters_[lbl - 1].cells.push_back( Cell(x, y, lbl) );
                }
            }
        }

        //!< Küme özelliklerini hesapla
        for (auto& cluster : clusters_) 
        {
            if ( (int)cluster.cells.size() >= MIN_CLUSTER_SIZE ) 
            {
                calculateClusterProperties(cluster);
            }
        }

        //!< Küçük kümeleri temizle
        clusters_.erase(
            std::remove_if(clusters_.begin(), clusters_.end(),
                [&](const FrontierCluster& c) {
                    return (int)c.cells.size() < MIN_CLUSTER_SIZE;
                }),
            clusters_.end()
        );
    }

    /**
     * @brief Bir frontier kümesinin centroid (merkez noktası) ve 'information_potential' değerini hesaplar.
     * @param cluster Frontier kümesi referansı
     */
    void calculateClusterProperties(FrontierCluster& cluster) 
    {
        //!< Centroid
        double sum_x = 0.0, sum_y = 0.0;
        for (auto& cell : cluster.cells) 
        {
            sum_x += cell.x;
            sum_y += cell.y;
        }
        double cx = (sum_x / cluster.cells.size());
        double cy = (sum_y / cluster.cells.size());

        cluster.centroid.x = cx * current_map_.info.resolution + current_map_.info.origin.position.x;
        cluster.centroid.y = cy * current_map_.info.resolution + current_map_.info.origin.position.y;
        cluster.centroid.z = 0.0;

        cluster.size = cluster.cells.size();
        cluster.assigned = false;

        //!< Info gain
        cluster.information_potential = calculateInformationPotential(cluster);
    }

    /**
     * @brief Kümenin çevresindeki bilinmeyen hücre sayısı üzerinden bir "bilgi potansiyeli" değeri döndürür.
     * @param cluster Değerlendirilecek frontier kümesi
     * @return Kümeye ait tahmini bilgi kazanımı
     */
    double calculateInformationPotential(const FrontierCluster& cluster) 
    {
        //!< Küme civarındaki -1 hücre sayısı mantigi
        int unknown_cells = 0;
        std::set<std::pair<int,int>> visited;

        for (auto& cell : cluster.cells) 
        {
            //!< 3x3'lük bir alan istenirse degistirilebilir fakat performans acisidan yeterli
            for (int dy = -3; dy <= 3; dy++) 
            {
                for (int dx = -3; dx <= 3; dx++) 
                {
                    int nx = cell.x + dx;
                    int ny = cell.y + dy;
                    auto pos = std::make_pair(nx, ny);

                    if (visited.count(pos) == 0) 
                    {
                        visited.insert(pos);
                        //!<Sınır içinde mi?
                        if (nx >= 0 && nx < (int)current_map_.info.width &&
                            ny >= 0 && ny < (int)current_map_.info.height) 
                        {
                            int idx = ny * current_map_.info.width + nx;
                            if (current_map_.data[idx] == -1) 
                            {
                                unknown_cells++;
                            }
                        }
                    }
                }
            }
        }
        //!< Unknown hücre adedini hucre alaniyla carpiyoruz
        return unknown_cells * current_map_.info.resolution * current_map_.info.resolution;
    }

   /**
     * @brief Bir hedefe giderken planın geçerli olup olmadığını (mesafesi vs.) kontrol eder.
     * @param robot İlgili robot
     * @param target Robotun gitmesi istenen nokta
     * @return Plan geçerliyse true, yoksa false
     */
    bool isPlanValid(Robot& robot, const geometry_msgs::Point& target) 
    {
        nav_msgs::GetPlan srv;
        srv.request.start = robot.current_pose;

        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();
        goal.pose.position = target;
        goal.pose.orientation.w = 1.0;

        srv.request.goal = goal;
        srv.request.tolerance = 0.5;

        //!< plan_client çağrısı
        if (robot.plan_client.call(srv)) 
        {
            if (!srv.response.plan.poses.empty()) 
            {
                double path_length = calculatePathLength(srv.response.plan.poses);
                ROS_DEBUG("Plan for robot %s to (%.2f, %.2f): length=%.2f",
                      robot.name.c_str(), target.x, target.y, path_length);
                return (path_length < MAX_PLAN_DISTANCE);
            }
            else
            {
                ROS_WARN("Plan empty for robot %s to (%.2f, %.2f).", 
                     robot.name.c_str(), target.x, target.y);
            }
        }
        else
        {
            ROS_ERROR("Failed to call plan service for robot %s", robot.name.c_str());
        }
        return false;
    }

    /**
     * @brief Plan sonucundaki pose dizisinin toplam uzunluğunu (Öklid) hesaplar.
     * @param poses Plan içindeki konum dizisi
     * @return Toplam mesafe
     */
    double calculatePathLength(const std::vector<geometry_msgs::PoseStamped>& poses) 
    {
        if (poses.size() < 2) 
        {
            return 0.0;
        }
        double total = 0.0;
        for (size_t i = 1; i < poses.size(); ++i) 
        {
            double dx = poses[i].pose.position.x - poses[i-1].pose.position.x;
            double dy = poses[i].pose.position.y - poses[i-1].pose.position.y;
            double dz = poses[i].pose.position.z - poses[i-1].pose.position.z;
            total += std::sqrt(dx*dx + dy*dy + dz*dz);
        }
        return total;
    }

    /**
     * @brief Robot ve atanmamış frontier kümeleri için plan sorguları yapıp,
     *        path uzunluklarını önbelleğe kaydeder.
     * @param available_robots Müsait robotların listesi
     */
    void calculateAllPaths(const std::vector<Robot*>& available_robots) 
    {
        if (!path_cache_.isValid()) 
        {
            //!< Cache süresi doldu, temizle
            path_cache_.clear();
            path_cache_.last_update = ros::Time::now();

            for (auto* robot : available_robots) 
            {
                for (size_t i = 0; i < clusters_.size(); i++) 
                {
                    if (clusters_[i].assigned)
                    {
                        continue;
                    }

                    geometry_msgs::Point target = findAccessibleTarget(clusters_[i], *robot);
                    
                    nav_msgs::GetPlan srv;
                    srv.request.start = robot->current_pose;

                    geometry_msgs::PoseStamped goal;
                    goal.header.frame_id = "map";
                    goal.header.stamp = ros::Time::now();
                    goal.pose.position = target;
                    goal.pose.orientation.w = 1.0;

                    srv.request.goal = goal;
                    srv.request.tolerance = 0.5;

                    double path_length = std::numeric_limits<double>::max();
                    if (robot->plan_client.call(srv)) 
                    {
                        if (!srv.response.plan.poses.empty()) 
                        {
                            path_length = calculatePathLength(srv.response.plan.poses);
                        }
                    }

                    PathCache::PathKey key{robot->name, (int)i};
                    path_cache_.path_lengths[key] = path_length;
                }
            }
        }
    }

    /**
     * @brief Centroid planlanamazsa, kümedeki hücrelerden en yakına plan deneyerek 
     *        erişilebilir bir hedef bulur.
     * @param cluster Frontier kümesi
     * @param robot İlgili robot
     * @return Erişilebilir hedef noktası
     */
    geometry_msgs::Point findAccessibleTarget(const FrontierCluster& cluster, Robot& robot) 
    {
        //!< Önce centroid
        if (isPlanValid(robot, cluster.centroid)) 
        {
            return cluster.centroid;
        }

        //!< Centroid erişilemezse, alternatif hücreler
        std::vector<std::pair<geometry_msgs::Point, double>> candidates;
        candidates.reserve(cluster.cells.size());

        for (auto& c : cluster.cells) 
        {
            geometry_msgs::Point pt;
            pt.x = c.x * current_map_.info.resolution + current_map_.info.origin.position.x;
            pt.y = c.y * current_map_.info.resolution + current_map_.info.origin.position.y;
            pt.z = 0.0;

            double dx = pt.x - robot.current_pose.pose.position.x;
            double dy = pt.y - robot.current_pose.pose.position.y;
            double dist = std::sqrt(dx*dx + dy*dy);
            candidates.push_back({pt, dist});
        }
        //!< En yakından uzağa sırala
        std::sort(candidates.begin(), candidates.end(),
                  [](auto& a, auto& b){ return a.second < b.second; });

        //!< En yakın MAX_ALTERNATIVE_TARGETS nokta içinde plan denenir
        int limit = std::min((int)candidates.size(), MAX_ALTERNATIVE_TARGETS);
        for (int i = 0; i < limit; i++) 
        {
            if (isPlanValid(robot, candidates[i].first)) 
            {
                return candidates[i].first;
            }
        }

        //!< Hiçbiri yoksa en yakını döndür
        return candidates[0].first;
    }

    /**
     * @brief Önbellekte kayıtlı mesafeyi döndürür, yoksa sonsuz.
     * @param robot İlgili robot
     * @param cluster Frontier kümesi
     * @param cluster_idx Kümenin indeks değeri
     * @return Önbellekte varsa path uzunluğu, yoksa sonsuz
     */
    double getPathLength(const Robot& robot, const FrontierCluster& cluster, int cluster_idx) 
    {
        //!< Cache'den al
        PathCache::PathKey key{robot.name, cluster_idx};
        auto it = path_cache_.path_lengths.find(key);
        if (it != path_cache_.path_lengths.end()) 
        {
            return it->second;
        }
        //!< Yoksa sonsuz
        return std::numeric_limits<double>::max();
    }

    /**
     * @brief Müsait robotları toplayıp, her robotun en yüksek utility'li kümeye atamasını yapar.
     */
    void assignTargets() 
    {
        ROS_INFO("Assigning targets to available robots...");
        std::vector<Robot*> available_robots;
        for (auto& robot : robots_) 
        {
            if (robot.is_available) 
            {   
                ROS_INFO("Uygun robot var");
                available_robots.push_back(&robot);
            }
        }
        if (available_robots.empty() || clusters_.empty()) {
            ROS_INFO("ATAMA OLMADI");
            return;
        }

        //!<Cache güncelle
        calculateAllPaths(available_robots);
        ROS_INFO("PATHLER HESAPLANIYOR");
        //!< Utility matrisi
        std::vector<std::vector<double>> utility_matrix;
        utility_matrix.reserve(available_robots.size());

        for (auto* robot : available_robots) 
        {
            std::vector<double> robot_utilities;
            for (size_t i = 0; i < clusters_.size(); i++) 
            {
                if (!clusters_[i].assigned) 
                {
                    double path_len = getPathLength(*robot, clusters_[i], (int)i);
                    //!<  Bir frontier kümesi için robotun fayda (utility) değerini hesaplayalim 
                    //!< Basitçe: utility = information_potential - ALPHA * path_length
                    double utility = clusters_[i].information_potential - ALPHA * path_len;
                    robot_utilities.push_back(utility);
                }
                else 
                {
                    //!<Atanmış cluster
                    robot_utilities.push_back(-999999.0);
                }
            }
            utility_matrix.push_back(robot_utilities);
        }

        //!< Greedy atama
        for (size_t i = 0; i < available_robots.size(); i++) 
        {
            double best_utility = MIN_UTILITY;
            size_t best_cluster_idx = SIZE_MAX;
            ROS_DEBUG("Checking clusters for robot: %s", available_robots[i]->name.c_str());
            
            for (size_t j = 0; j < clusters_.size(); j++) 
            {
                if (!clusters_[j].assigned && utility_matrix[i][j] > best_utility) 
                {
                    best_utility = utility_matrix[i][j];
                    best_cluster_idx = j;
                    ROS_DEBUG(" --> Cluster %zu, utility=%.2f", j, best_utility);
                }
            }

            if (best_cluster_idx != SIZE_MAX) 
            {
                clusters_[best_cluster_idx].assigned = true;
                geometry_msgs::Point target = findAccessibleTarget(clusters_[best_cluster_idx], *available_robots[i]);
                ROS_INFO("Robot %s --> cluster %zu (utility=%.2f)",available_robots[i]->name.c_str(),
                                                                   best_cluster_idx, best_utility);
                sendGoal(*available_robots[i], target);
            }
        }

        ROS_INFO("Clusters size: %zu", clusters_.size());
        ROS_INFO("Available robots: %zu", available_robots.size());

        //!< loglama için bastiralim
        for (size_t j = 0; j < clusters_.size(); j++) 
        {
            ROS_INFO("Cluster %zu: assigned=%d, info_potential=%.2f", 
                    j, 
                    clusters_[j].assigned, 
                    clusters_[j].information_potential);
        }

        for (size_t i = 0; i < available_robots.size(); i++) 
        {
            for (size_t j = 0; j < clusters_.size(); j++) 
            {
                ROS_INFO("Robot %s, Cluster %zu: utility=%.2f", 
                        available_robots[i]->name.c_str(), 
                        j, 
                        utility_matrix[i][j]);
            }
        }
    }

    /**
     * @brief move_base'e hedef gönderir, sonuç bekler, zaman aşımı durumunda iptal eder.
     * @param robot Hedef yollanacak robot
     * @param target Hedef nokta
     */
    void sendGoal(Robot& robot, const geometry_msgs::Point& target) 
    {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position = target;
        
        //!< Robotu hedef yöne döndürmek için yaw
        double dx = target.x - robot.current_pose.pose.position.x;
        double dy = target.y - robot.current_pose.pose.position.y;
        double yaw = std::atan2(dy, dx);

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        goal.target_pose.pose.orientation = tf2::toMsg(q);

        robot.is_available = false;
        robot.action_client->sendGoal(
            goal,
            boost::bind(&MultiRobotExplorer::doneCallback, this, _1, _2, &robot),
            boost::bind(&MultiRobotExplorer::activeCallback, this),
            boost::bind(&MultiRobotExplorer::feedbackCallback, this, _1)
        );
    }
};

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "multi_robot_explorer");
    MultiRobotExplorer explorer;
    ros::spin();
    return 0;
}