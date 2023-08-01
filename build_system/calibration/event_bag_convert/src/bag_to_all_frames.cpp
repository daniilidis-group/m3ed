// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Edited by Kenneth Chaney for dataset support
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <event_array_msgs/EventArray.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <unistd.h>

#include <memory>

#include <simple_image_recon/approx_reconstructor.hpp>
#include <simple_image_recon/frame_handler.hpp>

#include <yaml-cpp/yaml.h>
#include <event_bag_convert/prettyprint.hpp>

using event_array_msgs::EventArray;
using sensor_msgs::Image;
using sensor_msgs::CompressedImage;
using ApproxRecon = simple_image_recon::ApproxReconstructor<
  EventArray, EventArray::ConstPtr, Image, Image::ConstPtr>;

int64_t retime_ovc(int64_t time_nsec, int64_t offset, double skew){
  int64_t new_time = time_nsec-offset;

  int64_t skew_change = (new_time / 1e9) * skew;
  new_time += skew_change;

  // new_time /= 2500000;
  // new_time *= 2500000;

  return new_time;
}

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "bag_to_frames -i input_bag -o output_bag -t time_sync_file"
               "[-c cutoff_period] [-d image_decimator]"
            << std::endl;
}

class OutBagWriter : public simple_image_recon::FrameHandler<Image::ConstPtr>
{
public:
  explicit OutBagWriter(const std::string & bagName)
  {
    outBag_.open(bagName, rosbag::bagmode::Write);
  }
  void frame(const Image::ConstPtr & img, const std::string & topic) override
  {
    outBag_.write(topic, img->header.stamp, img);
    numFrames_++;
    if (numFrames_ % 100 == 0) {
      std::cout << "wrote " << numFrames_ << " frames " << std::endl;
    }
  }

  rosbag::Bag outBag_;
  size_t numFrames_{0};
};

using simple_image_recon::ApproxReconstructor;

int main(int argc, char ** argv)
{
  int opt;
  std::string inBagName;
  std::string outBagName;
  std::string timeSyncFileName;

  std::vector<std::string> eventTopics{"/prophesee/left/events","/prophesee/right/events"};
  std::vector<std::string> ovcTopics{"/ovc/left/image_mono/compressed","/ovc/right/image_mono/compressed","/ovc/rgb/image_color/compressed"};

  int cutoff_period(45);

  int image_decimator(1);

  while ((opt = getopt(argc, argv, "i:o:O:t:c:d:h")) != -1) {
    switch (opt) {
      case 'i':
        inBagName = optarg;
        break;
      case 'o':
        outBagName = optarg;
        break;
      case 't':
        timeSyncFileName = optarg;
        break;
      case 'c':
        cutoff_period = atoi(optarg);
        break;
      case 'd':
        image_decimator = atoi(optarg);
        break;
      case 'h':
        usage();
        return (-1);
        break;
      default:
        std::cout << "unknown option: " << opt << std::endl;
        usage();
        return (-1);
        break;
    }
  }

  if (inBagName.empty()) {
    std::cout << "missing input bag file name!" << std::endl;
    usage();
    return (-1);
  }

  if (outBagName.empty()) {
    std::cout << "missing output bag file name!" << std::endl;
    usage();
    return (-1);
  }

  if (timeSyncFileName.empty()) {
    std::cout << "missing time sync file name!" << std::endl;
    usage();
    return (-1);
  }

  std::map<std::string, int64_t> offsets;
  std::map<std::string, double> skews;

  std::cout << "Opening " << timeSyncFileName << std::endl;

  // This file is in micro-seconds
  YAML::Node time_sync_config = YAML::LoadFile(timeSyncFileName);

  auto ovc_topic_node = time_sync_config["/ovc/pps_cb"];
  int vnav_offset = 2500*1000*ovc_topic_node["decimator"]["round_offset"].as<int>();

  offsets["ovc"] = 1000*ovc_topic_node["correction"]["offset"].as<int64_t>() + vnav_offset;
  skews["ovc"] = 1000*ovc_topic_node["correction"]["skew"].as<double>();

  auto left_topic_node = time_sync_config[eventTopics[0]];
  offsets[eventTopics[0]] = 1000*left_topic_node["correction"]["offset"].as<int64_t>() + vnav_offset;
  skews[eventTopics[0]] = 1000*left_topic_node["correction"]["skew"].as<double>();

  auto right_topic_node = time_sync_config[eventTopics[1]];
  offsets[eventTopics[1]] = 1000*right_topic_node["correction"]["offset"].as<int64_t>() + vnav_offset;
  skews[eventTopics[1]] = 1000*right_topic_node["correction"]["skew"].as<double>();

  std::cout << "Retrieved time synchronizations" << std::endl;
  std::cout << "- vnav phase offset " << vnav_offset << std::endl;
  std::cout << "- Offsets : " << offsets << std::endl;
  std::cout << "- Skews : " << skews << std::endl;
  std::cout << "" << std::endl;

  std::cout << "Settings" << std::endl;
  std::cout << "- Image Decimator " << image_decimator << std::endl;
  std::cout << "- FilterCutoff " << cutoff_period << std::endl;

  OutBagWriter writer(outBagName);

  // Grab Frame times from original bag and copy images from OVC over
  std::vector<int64_t> ovc_times;

  std::cout << "Opening " << inBagName << std::endl;
  rosbag::Bag inBag;
  inBag.open(inBagName, rosbag::bagmode::Read);
  std::cout << "- Opened" << std::endl;

  ros::Time first_image_time;
  bool set_first_image = false;

  std::cout << "Get frame times to write to bag" << std::endl;
  rosbag::View view_primary(inBag, rosbag::TopicQuery(std::vector<std::string>{ovcTopics[2]}));
  int image_decimator_counter(0);
  for (const rosbag::MessageInstance & msg : view_primary) {
    auto m = msg.instantiate<CompressedImage>();
    if(!set_first_image){
      first_image_time = m->header.stamp;
      if( static_cast<double>(first_image_time.toNSec()) - static_cast<double>(offsets["ovc"]) < 0 ) {
        continue;
      }else{
        set_first_image = true;
      }
    }
    if(set_first_image && image_decimator_counter++ % image_decimator == 0){
      ovc_times.emplace_back( offsets["ovc"] + retime_ovc(m->header.stamp.toNSec(), offsets["ovc"], skews["ovc"]) );
    }
  }

  std::cout << "- Requesting " << ovc_times.size() << " frames" << std::endl;
  std::cout << "- Start : " << ovc_times[0] << std::endl;
  std::cout << "- End : " << ovc_times[ovc_times.size()-1] << std::endl;
  std::cout << "- Start (rel) : " << ovc_times[0]-offsets["ovc"] << std::endl;
  std::cout << "- End (rel) : " << ovc_times[ovc_times.size()-1]-offsets["ovc"] << std::endl;

  std::cout << "Copying OVC frames" << std::endl;
  rosbag::View view_ovc(inBag, rosbag::TopicQuery(ovcTopics));
  for (const rosbag::MessageInstance & msg : view_ovc) {
    auto m = msg.instantiate<CompressedImage>();
    auto retimed = offsets["ovc"] + retime_ovc(m->header.stamp.toNSec(), offsets["ovc"], skews["ovc"]);

    auto find_res = std::find(ovc_times.begin(),
                              ovc_times.end(),
                              retimed);
    if(find_res != ovc_times.end()){
      m->header.stamp.fromNSec( retimed );
      writer.outBag_.write(msg.getTopic(), m->header.stamp, m);
    }
  }

  std::cout << "Building Reconstructors" << std::endl;
  const double fillRatio = 0.6;
  const int tileSize = 2;
  std::unordered_map<std::string, ApproxRecon> recons;
  for (size_t i = 0; i < eventTopics.size(); i++) {

    std::cout << " Final topic offset " << eventTopics[i] << " : " << offsets[eventTopics[i]] << std::endl;
    recons.insert(
      {eventTopics[i], ApproxRecon(
                      &writer, eventTopics[i], cutoff_period, 0.1, fillRatio,
                      tileSize, offsets["ovc"]-offsets[eventTopics[i]], ovc_times)});
  }

  size_t numMessages(0);
  rosbag::View view_events(inBag, rosbag::TopicQuery(eventTopics));
  for (const rosbag::MessageInstance & msg : view_events) {
    auto it = recons.find(msg.getTopic());
    if (it != recons.end()) {
      auto m = msg.instantiate<EventArray>();
      it->second.processMsg(m);
      numMessages++;
    }
  }
  std::cout << "processed " << numMessages << " number of messages"
            << std::endl;

  return (0);
}
