#include <cmath>

#include <array>
#include <optional>
#if __cplusplus >= 202002L
#include <numbers>
#endif
#include <atomic>

#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>

#include <omni4/Twist2D.h>

#include <CRSLib/include/std_type.hpp>
#include <shirasu_md.hpp>

namespace omni4node
{
	using namespace CRSLib::IntegerTypes;

#if __cplusplus >= 202002L
	constexpr auto pi = std::numbers::pi;
#else
	constexpr auto pi = 3.14159265358979323846262338;
#endif

	inline constexpr double cast_shadow(const double x, const double y, const double theta) noexcept
	{
		return x * std::cos(theta) + y * std::sin(theta);
	}

	class Omni4Node final : public nodelet::Nodelet
	{
		/// @todo rosparam1で置き換える
		struct Config final
		{
			struct Wheel final
			{
				u32 id;
				double distance_from_center;
				double tangent_direction;
			};

			ros::NodeHandle& nh;
			std::array<Wheel, 4> wheels;
			double shirasu_pub_tim_duration = 1.0 / 1000;

			Config(ros::NodeHandle& nh):
				nh{nh}
			{
				constexpr double distance_from_center = 0.458;
				wheels[0] = Wheel{0x400, distance_from_center, 3.0 / 4.0 * pi};  // FR
				wheels[1] = Wheel{0x404, distance_from_center, 5.0 / 4.0 * pi};  // FL
				wheels[2] = Wheel{0x408, distance_from_center, 7.0 / 4.0 * pi};  // BL
				wheels[3] = Wheel{0x40C, distance_from_center, 1.0 / 4.0 * pi};  // BR
			}
		};

		struct Inner final
		{
			struct Wheel final
			{
				crs_lib::MotorDriver::ShirasuMd md;
				double distance_from_center;
				double tangent_direction;

				Wheel(crs_lib::MotorDriver::ShirasuMd&& md, const double distance_from_center, const double tangent_direction):
					md{std::move(md)},
					distance_from_center{distance_from_center},
					tangent_direction{tangent_direction}
				{}

				Wheel& operator=(Wheel&& other) noexcept
				{
					md = std::move(other.md);
					distance_from_center = other.distance_from_center;
					tangent_direction = other.tangent_direction;
					return *this;
				}
			};

			std::array<std::optional<Wheel>, 4> wheels{};

			std::array<std::atomic<double>, 4> targets{}; 

			ros::Subscriber body_twist_sub{};
			ros::Timer shirasu_pub_tim{};

			[[deprecated]] void adhoc_hochoc_adad_adhoc_initialize(const Config& config)
			{
				for(int i = 0; i < 4; ++i)
				{
					u32 id = config.wheels[i].id;
					double dist_from_center = config.wheels[i].distance_from_center;
					double tangent_direction = config.wheels[i].tangent_direction;
					wheels[i].emplace(crs_lib::MotorDriver::ShirasuMd{id, config.nh}, dist_from_center, tangent_direction);
				}
			}

			Inner(Config& config)
			{
				adhoc_hochoc_adad_adhoc_initialize(config);

				for(auto& wheel : wheels)
				{
					wheel->md.change_mode(crs_lib::MotorDriver::ShirasuMode::velocity);
				}

				body_twist_sub = config.nh.subscribe<omni4::Twist2D>("body_twist_velocity", 1, &Inner::body_twist_velocity_callback, this);
				shirasu_pub_tim = config.nh.createTimer(ros::Duration{config.shirasu_pub_tim_duration}, &Inner::shirasu_pub_tim_callback, this);
			}

			// thisポインタ登録、ムーブ不可
			Inner(Inner&&) = delete;
			Inner& operator=(Inner&&) = delete;

			void body_twist_velocity_callback(const omni4::Twist2D::ConstPtr& twist)
			{
				for(int i = 0; i < 4; ++i)
				{
					const double target = cast_shadow(twist->x, twist->y, wheels[i]->tangent_direction) + wheels[i]->distance_from_center * twist->z;
					targets[i] = target;
				}
			}

			void shirasu_pub_tim_callback(const ros::TimerEvent&)
			{
				for(int i = 0; i < 4; ++i)
				{
					wheels[i]->md.velocity_update(targets[i]);
				}
			}
		};

		std::optional<Inner> inner;

		void onInit() override
		{
			ros::NodeHandle nh = getNodeHandle();
			Config config{nh};

			inner.emplace(config);
		}
	};
}

PLUGINLIB_EXPORT_CLASS(omni4node::Omni4Node, nodelet::Nodelet);