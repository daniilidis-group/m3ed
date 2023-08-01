#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <ouster_ros/PacketMsg.h>
#include <sensor_msgs/Imu.h>

#include <ouster_ros/os_ros.h>

#include <map>

// Not defined in a header in ouster_ros...so copied
inline ros::Time to_ros_time(uint64_t ts) {
    ros::Time t;
    t.fromNSec(ts);
    return t;
}

// Not defined in a header in ouster_ros...so copied
inline ros::Time to_ros_time(std::chrono::nanoseconds ts) {
    return to_ros_time(ts.count());
}

std::string get_metadata(rosbag::Bag& bag){
  // Grab single message on /os_node/metadata
  std::string info;

  for(rosbag::MessageInstance const m: rosbag::View(bag, rosbag::TopicQuery("/os_node/metadata")))
  {
    std_msgs::String::ConstPtr str = m.instantiate<std_msgs::String>();
    if (str != nullptr)
      info = str->data;
  }

  return info;
}

sensor_msgs::ImuPtr PacketMsg_to_Imu(const ouster_ros::PacketMsg::ConstPtr& pm, ouster::sensor::sensor_info& info) {
  // Follow the logic from os_cloud_node.cpp in the ouster_ros driver
  // Specifically we use the sensor time code path
  auto pf = ouster::sensor::get_format(info);
  ros::Time msg_ts = to_ros_time(pf.imu_gyro_ts(pm->buf.data()));
  sensor_msgs::Imu imu_msg = ouster_ros::packet_to_imu_msg(*pm, msg_ts, "os_imu", pf);
  sensor_msgs::ImuPtr imu_msg_ptr =
      boost::make_shared<sensor_msgs::Imu>(imu_msg);
  return imu_msg_ptr;
}

sensor_msgs::PointCloud2Ptr LidarScan_to_PointCloud(ouster::LidarScan& ls, ouster::XYZLut& xyz_lut, ouster::sensor::sensor_info& info) {
  // Follow the logic from os_cloud_node.cpp in the ouster_ros driver
  // Specifically we use the sensor time code path
  sensor_msgs::PointCloud2Ptr pc_ptr;

  auto ts_v = ls.timestamp();
  auto idx = std::find_if(ts_v.data(), ts_v.data() + ts_v.size(),
      [](uint64_t h) { return h != 0; });
  if (idx == ts_v.data() + ts_v.size()) return pc_ptr;
  auto scan_ts = std::chrono::nanoseconds{ts_v(idx - ts_v.data())};
  ros::Time msg_ts = to_ros_time(scan_ts);

  uint32_t H = info.format.pixels_per_column;
  uint32_t W = info.format.columns_per_frame;
  ouster_ros::Cloud cloud{W,H};

  ouster_ros::scan_to_cloud(xyz_lut, scan_ts, ls, cloud, 0);

  sensor_msgs::PointCloud2 pc =
      ouster_ros::cloud_to_cloud_msg(cloud, msg_ts, "os_lidar");
  pc_ptr = boost::make_shared<sensor_msgs::PointCloud2>(pc);

  return pc_ptr;
}

int main(int argc, char *argv[])
{
  if(argc < 1){
    std::cout << "rosrun ouster_bag_convert ouster_bag_converter <read_bag> <write_bag>" << std::endl;
    return 1;
  }

  std::string read_bag_fn{argv[1]};
  std::string write_bag_fn{argv[2]};

  std::cout << "=== Opening ===" << std::endl;
  std::cout << " - Reading: " << read_bag_fn << std::endl;
  std::cout << " - Writing: " << write_bag_fn << std::endl;

  // Read bag with topics to pull packets from
  rosbag::Bag read_bag;
  read_bag.open(read_bag_fn, rosbag::bagmode::Read);
  const std::string read_topic_lidar{"/os_node/lidar_packets"};
  const std::string read_topic_imu{"/os_node/imu_packets"};

  // Write bag with topics to write to
  rosbag::Bag write_bag;
  write_bag.open(write_bag_fn, rosbag::bagmode::Write);
  const std::string write_topic_lidar{"/os_node/points"};
  const std::string write_topic_imu{"/os_node/imu"};

  // Grab Ouster metadata from bag
  std::string ouster_metadata = get_metadata(read_bag);

  ouster::sensor::sensor_info info = ouster::sensor::parse_metadata(ouster_metadata);
  ouster::XYZLut xyz_lut = ouster::make_xyz_lut(info);
  uint32_t H = info.format.pixels_per_column;
  uint32_t W = info.format.columns_per_frame;
  std::unique_ptr<ouster::ScanBatcher> scan_batcher = std::make_unique<ouster::ScanBatcher>(info);
  ouster::LidarScan ls{W, H, info.format.udp_profile_lidar};

  std::cout << "Starting loop" << std::endl;
  int i = 0;

  for(rosbag::MessageInstance const m: rosbag::View(read_bag, rosbag::TopicQuery({read_topic_lidar, read_topic_imu})))
  {
    ouster_ros::PacketMsg::ConstPtr pm = m.instantiate<ouster_ros::PacketMsg>();

    if(m.getTopic() == read_topic_imu) {
      auto imu_ptr = PacketMsg_to_Imu(pm, info);
      if(!imu_ptr)continue;

      write_bag.write(write_topic_imu, m.getTime(), imu_ptr);

    } else if(m.getTopic() == read_topic_lidar) {
      if(!(*scan_batcher)(pm->buf.data(), ls))continue; // Not a full scan
      auto pc_ptr = LidarScan_to_PointCloud(ls, xyz_lut, info);
      if(!pc_ptr)continue;

      write_bag.write(write_topic_lidar, m.getTime(), pc_ptr);
    }

    if (i++ % 1000 == 0) std::cout << "+" << std::flush;
  }
  std::cout << std::endl << "Done" << std::endl;

  read_bag.close();
  write_bag.close();

  return 0;
}
