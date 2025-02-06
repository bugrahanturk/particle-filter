#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <vector>
#include <iostream>

using namespace std;

//!< Parcacik yapisi
struct Parcacik 
{
    float x, y, theta;
};

vector<Parcacik> parcaciklar;

//!< Publisher tanimlamalari
ros::Publisher hesaplanan_pose_pub;
ros::Publisher robot_marker_pub;
ros::Publisher parcacik_marker_pub;

//!< Gurultu modelinde kullanilan parametrelerin tanimlanmasi
float alpha1 = 0.001, alpha2 = 0.001, alpha3 = 0.001, alpha4 = 0.001;

//!< Onceki odometri degerleri
float onceki_x = 0.0, onceki_y = 0.0, onceki_theta = 0.0;
bool ilk_odom_verisi_mi = true;

//!< Ucgen dagilimina gore ornekleme fonksiyonu
float Ucgen_Dagilim_Orneklemesi(float b) 
{
    //!<  (2 * b) - b işlemi, rastgele degeri dogru araliga ([-b, b]) olceklemek ve kaydirmak icin
    float sonuc = (sqrt(6) / 2) * (((float)rand() / RAND_MAX) * (2 * b) - b + ((float)rand() / RAND_MAX) * (2 * b) - b);
    return sonuc; 
}

//!< Parcaciklari MarkerArray formatinda yayinla
void Parcacik_Markerlarinin_Yayinla() 
{
    visualization_msgs::MarkerArray marker_array;

    for (size_t i = 0; i < parcaciklar.size(); ++i) 
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp    = ros::Time::now();
        marker.ns              = "particles";
        marker.id              = i;
        marker.type            = visualization_msgs::Marker::SPHERE;
        marker.action          = visualization_msgs::Marker::ADD;

        //!< Pozisyon
        marker.pose.position.x = parcaciklar[i].x;
        marker.pose.position.y = parcaciklar[i].y;
        marker.pose.position.z = 0.0;

        //!< Orientation
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        //!< Boyut
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        //!< Renk
        marker.color.r = 1.0f; 
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.5f; 

        marker_array.markers.push_back(marker);
    }

    parcacik_marker_pub.publish(marker_array);
}

//!< Robotun tahmini konumunu Marker formatinda yayinla
void Robot_Marker_Yayinla(float x, float y, float theta) 
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp    = ros::Time::now();
    marker.ns              = "robot";
    marker.id              = 0;
    marker.type            = visualization_msgs::Marker::ARROW;
    marker.action          = visualization_msgs::Marker::ADD;

    //!< Pozisyon
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.0;

    //!< Yonelim
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    // Boyut ve renk
    marker.scale.x = 0.5;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    robot_marker_pub.publish(marker);
}

void Odom_Callback(const nav_msgs::Odometry::ConstPtr &msg) 
{
    float simdiki_x = msg->pose.pose.position.x;
    float simdiki_y = msg->pose.pose.position.y;
    float tahmini_x = 0.0, tahmini_y = 0.0, tahmini_theta = 0.0;

    //!< Quaternion'dan Yaw elde et
    geometry_msgs::Quaternion quat = msg->pose.pose.orientation;
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    float simdiki_theta = yaw;

    //!< Ilk odometri mesaji icin baslangic degerlerini ayarla
    if (true == ilk_odom_verisi_mi) 
    {
        onceki_x           = simdiki_x;
        onceki_y           = simdiki_y;
        onceki_theta       = simdiki_theta;
        ilk_odom_verisi_mi = false;
        return;
    }

    //!< Hareket parametrelerini hesapla
    float delta_x     = simdiki_x - onceki_x;
    float delta_y     = simdiki_y - onceki_y;
    float delta_rot1  = atan2(delta_y, delta_x) - onceki_theta;
    float delta_trans = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
    float delta_rot2  = simdiki_theta - onceki_theta - delta_rot1;

    //!< Parcaciklari guncelle
    for (auto &particle : parcaciklar) 
    {
        float delta_rot1_sonraki  = delta_rot1 + Ucgen_Dagilim_Orneklemesi(alpha1 * pow(delta_rot1, 2) + alpha2 * pow(delta_trans, 2));
        float delta_trans_sonraki = delta_trans + Ucgen_Dagilim_Orneklemesi(alpha3 * pow(delta_trans, 2) + alpha4 * (pow(delta_rot1, 2) + pow(delta_rot2, 2)));
        float delta_rot2_sonraki  = delta_rot2 + Ucgen_Dagilim_Orneklemesi(alpha1 * pow(delta_rot2, 2) + alpha2 * pow(delta_trans, 2));

        //!< Parcaciklarin yeni pozisyonu kendilerinden (particle.x, particle.y, particle.theta) guncelleniyor
        //!< cunku parcaciklarin bagimsiz olarak hareket etmesini simule ediyoruz. 
        particle.x     += delta_trans_sonraki * cos(particle.theta + delta_rot1_sonraki);
        particle.y     += delta_trans_sonraki * sin(particle.theta + delta_rot1_sonraki);
        particle.theta += delta_rot1_sonraki + delta_rot2_sonraki;
    }

    //!< Ortalama tahmini konumu hesapla
    for (const auto &particle : parcaciklar)
     {
        tahmini_x     += particle.x;
        tahmini_y     += particle.y;
        tahmini_theta += particle.theta;
    }

    tahmini_x     /= parcaciklar.size();
    tahmini_y     /= parcaciklar.size();
    tahmini_theta /= parcaciklar.size();

    //!< Marker'lari yayınla
    Parcacik_Markerlarinin_Yayinla();
    Robot_Marker_Yayinla(tahmini_x, tahmini_y, tahmini_theta);

    //!< Tahmini konumu yayinla (buraya publish olanlar direkt konumu dinleyebilsin diye)
    geometry_msgs::Pose2D tahmini_konum;
    tahmini_konum.x     = tahmini_x;
    tahmini_konum.y     = tahmini_y;
    tahmini_konum.theta = tahmini_theta;

    hesaplanan_pose_pub.publish(tahmini_konum);

    //!< Tahmini konumu konsola yazdır
    ROS_INFO("Tahmini Robot Konumu: X=%.2f, Y=%.2f, Theta=%.2f (rad)", tahmini_x, tahmini_y, tahmini_theta);

    //!< Onceki odometriyi güncelle
    onceki_x     = simdiki_x;
    onceki_y     = simdiki_y;
    onceki_theta = simdiki_theta;
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "particle_filter_node");
    ros::NodeHandle n("~");
    int numParticles;

    if (!(n.getParam("numParticles",numParticles)))
        numParticles = 100;

    std::cout << "numParticles" << numParticles <<std::endl;

    //!< Parcaciklari rastgele baslatalim, odometri verisiyle guncelleyecegiz
    parcaciklar.resize(numParticles);
    for (auto &particle : parcaciklar) 
    {
        particle.x     = onceki_x + ((float)rand() / RAND_MAX) * 0.5 - 0.25; //!< Rastgele -0.25 ile 0.25 arasinda
        particle.y     = onceki_y + ((float)rand() / RAND_MAX) * 0.5 - 0.25;
        particle.theta = onceki_theta + ((float)rand() / RAND_MAX) * M_PI / 18 - M_PI / 36; //!< Kucuk bir aci sapmasi
    }

    //!< Publisher, Subs
    hesaplanan_pose_pub      = n.advertise<geometry_msgs::Pose2D>("estimated_pose", 1);
    robot_marker_pub         = n.advertise<visualization_msgs::Marker>("robot_marker", 1);
    parcacik_marker_pub      = n.advertise<visualization_msgs::MarkerArray>("particles_markers", 1);
    ros::Subscriber odom_sub = n.subscribe("/odom", 1, Odom_Callback);

    //!< ROS dongusu
    ros::spin();

    return 0;
}
