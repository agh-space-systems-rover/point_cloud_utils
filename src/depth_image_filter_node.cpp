#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>

namespace point_cloud_utils {

class DepthImageFilter : public rclcpp::Node {
  public:
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr    pub;

	DepthImageFilter(const rclcpp::NodeOptions &options)
	    : Node("depth_image_filter", options) {
		rclcpp::QoS img_qos = rclcpp::QoS(1);
		img_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
		img_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

		sub = create_subscription<sensor_msgs::msg::Image>(
		    "image_raw",
		    img_qos,
		    std::bind(&DepthImageFilter::callback, this, std::placeholders::_1)
		);
		pub = create_publisher<sensor_msgs::msg::Image>("image_filtered", img_qos);
	}

	void callback(const sensor_msgs::msg::Image::ConstSharedPtr &input) {
		if (input->encoding != sensor_msgs::image_encodings::TYPE_16UC1) {
			RCLCPP_WARN_THROTTLE(
			    get_logger(),
			    *get_clock(),
			    5000,
			    "Expected image encoding '%s', got '%s'",
			    sensor_msgs::image_encodings::TYPE_16UC1,
			    input->encoding.c_str()
			);
			return;
		}

		constexpr size_t pixel_size = sizeof(uint16_t);
		const size_t     min_step   = input->width * pixel_size;
		if (input->step < min_step || input->data.size() < input->step * input->height) {
			RCLCPP_WARN_THROTTLE(
			    get_logger(),
			    *get_clock(),
			    5000,
			    "Invalid 16UC1 image layout: width=%u height=%u step=%u data=%zu",
			    input->width,
			    input->height,
			    input->step,
			    input->data.size()
			);
			return;
		}

		const size_t width      = input->width;
		const size_t height     = input->height;
		const size_t pixel_count = width * height;

		std::vector<uint16_t> depth(pixel_count, 0);
		std::vector<uint8_t>  valid(pixel_count, 0);
		for (size_t row = 0; row < height; row++) {
			const size_t input_row_offset = row * input->step;
			const size_t mask_row_offset  = row * width;
			for (size_t col = 0; col < width; col++) {
				const size_t   input_offset = input_row_offset + col * pixel_size;
				const size_t   index        = mask_row_offset + col;
				const uint16_t value =
				    read_uint16(input->data, input_offset, input->is_bigendian != 0);
				depth[index] = value;
				valid[index] = value == 0 ? 0 : 1;
			}
		}

		std::vector<uint8_t> connected(pixel_count, 0);
		std::vector<size_t>  queue;
		queue.reserve(pixel_count);

		const size_t seed_height = std::max<size_t>(1, height / 10);
		const size_t seed_row    = height - seed_height;
		for (size_t row = seed_row; row < height; row++) {
			const size_t row_offset = row * width;
			for (size_t col = 0; col < width; col++) {
				const size_t index = row_offset + col;
				if (valid[index] != 0 && connected[index] == 0) {
					connected[index] = 1;
					queue.push_back(index);
				}
			}
		}

		for (size_t queue_index = 0; queue_index < queue.size(); queue_index++) {
			const size_t index = queue[queue_index];
			const size_t row   = index / width;
			const size_t col   = index % width;

			try_visit(index - width, row > 0, valid, connected, queue);
			try_visit(index + width, row + 1 < height, valid, connected, queue);
			try_visit(index - 1, col > 0, valid, connected, queue);
			try_visit(index + 1, col + 1 < width, valid, connected, queue);
		}

		sensor_msgs::msg::Image output;
		output.header       = input->header;
		output.height       = input->height;
		output.width        = input->width;
		output.encoding     = sensor_msgs::image_encodings::TYPE_16UC1;
		output.is_bigendian = false;
		output.step         = min_step;
		output.data.resize(output.step * output.height);

		for (size_t row = 0; row < height; row++) {
			const size_t mask_row_offset   = row * width;
			const size_t output_row_offset = row * output.step;
			for (size_t col = 0; col < width; col++) {
				const size_t   index          = mask_row_offset + col;
				const uint16_t filtered_value = connected[index] != 0 ? depth[index] : 0;
				std::memcpy(
				    &output.data[output_row_offset + col * pixel_size],
				    &filtered_value,
				    pixel_size
				);
			}
		}

		pub->publish(output);
	}

  private:
	static void try_visit(
	    size_t                     index,
	    bool                       in_bounds,
	    const std::vector<uint8_t> &valid,
	    std::vector<uint8_t>       &connected,
	    std::vector<size_t>        &queue
	) {
		if (in_bounds && valid[index] != 0 && connected[index] == 0) {
			connected[index] = 1;
			queue.push_back(index);
		}
	}

	static uint16_t read_uint16(
	    const std::vector<uint8_t> &data, size_t offset, bool bigendian
	) {
		if (bigendian) {
			return (static_cast<uint16_t>(data[offset]) << 8) |
			       static_cast<uint16_t>(data[offset + 1]);
		}

		return static_cast<uint16_t>(data[offset]) |
		       (static_cast<uint16_t>(data[offset + 1]) << 8);
	}
};

} // namespace point_cloud_utils

RCLCPP_COMPONENTS_REGISTER_NODE(point_cloud_utils::DepthImageFilter)
