#pragma once

/**
 * @file shirasu_md.hpp
 * @author Stew
 * @brief シラスのモタドラ。
 * メンバ関数の呼び出しは互いにスレッドセーフである。
 * @version 0.1
 * @date 2022-11-23
 */

#include <shared_mutex>
#include <string>
#include <atomic>
#include <memory>
#include <utility>

#include <ros/ros.h>

#include <adhoc_canplugins_onehalf/CanFrame.h>

#include <CRSLib/include/utility.hpp>
#include <CRSLib/include/std_type.hpp>

#include <adhoc_canplugins_onehalf/adhoc_can_plugins2_pack.hpp>

#include "motor_driver.hpp"

namespace crs_lib::MotorDriver
{
	enum class ShirasuMode : u8
	{
		disable = 0,
		default_ = 1,
		homing = 2,
		reserved,
		current = 4,
		velocity = 5,
		position = 6
	};

	namespace Implement
	{
		/// @brief シラスのモータードライバの実装部。
		class ShirasuMdInner final
		{
			u32 base_id;

		private:
			ShirasuMode mode{ShirasuMode::disable};
			mutable std::shared_mutex mutex{};
			std::atomic<bool> is_ajusting_zero{false};
			std::atomic<bool> is_entered_homing{false};
			std::atomic<float> current_target_velocity{std::nan("")};
			std::atomic<float> current_target_position{std::nan("")};
			
			ros::Publisher can_mode_pub;
			ros::Publisher can_target_pub;
			ros::Subscriber can_status_sub;

		public:
			ShirasuMdInner(const u32 base_id, ros::NodeHandle& nh):
				base_id{base_id},
				can_mode_pub{nh.advertise<adhoc_canplugins_onehalf::CanFrame>("can" + std::to_string(base_id), 10)},
				can_target_pub{nh.advertise<adhoc_canplugins_onehalf::CanFrame>("can" + std::to_string(base_id + 1), 1)},
				can_status_sub{nh.subscribe<adhoc_canplugins_onehalf::CanFrame>("can" + std::to_string(base_id + 3), 1, &ShirasuMdInner::update_status, this)}
			{}

			// thisポインタを登録しているため、ムーブ不可
			ShirasuMdInner(ShirasuMdInner&&) = delete;
			ShirasuMdInner& operator=(ShirasuMdInner&&) = delete;

			~ShirasuMdInner()
			{}

			void velocity_update(const float target)
			// [[expects: mode == ShirasuMode::velocity]]
			{
				std::shared_lock lock{mutex};
				if(mode != ShirasuMode::velocity)
				{
					/// @todo error report
					return;
				}
				current_target_velocity = target;
				update(target);
			}

			float get_velocity()
			{
				return current_target_velocity;
			}

			void position_update(const float target)
			// [[expects: mode == ShirasuMode::position]]
			{
				if(mode != ShirasuMode::position)
				{
					/// @todo error report
					return;
				}
				current_target_position = target;
				update(target);
			}

			float get_position()
			{
				return current_target_position;
			}

			void ajust_zero_point()
			{
				is_ajusting_zero = true;
				change_mode(ShirasuMode::homing);
			}

			bool finished_ajusting_zero()
			{
				return !is_ajusting_zero;
			}

			MotorDriver::Mode get_mode() const noexcept
			{
				std::shared_lock lock{mutex};
				switch(mode)
				{
				case ShirasuMode::disable:
					return MotorDriver::Mode::stop;

				case ShirasuMode::homing:
					return MotorDriver::Mode::powerless;

				case ShirasuMode::current:
				case ShirasuMode::velocity:
				case ShirasuMode::position:
					return MotorDriver::Mode::run;

				// default_, reservedにするな。さもなくばエラー。
				default:
					return MotorDriver::Mode::error;
				}
			}

			void to_stop()
			{
				change_mode(ShirasuMode::disable);
			}

			[[deprecated]] void to_powerless()
			{
				/// @todo 実装
			}

			/**
			 * @brief シラスモードを変更する。
			 * 
			 * @param mode ShirasuMode。チェックはしていないがdefault_とreservedには変更しないこと。
			 */
			void change_mode(const ShirasuMode mode)
			{
				adhoc_canplugins_onehalf::CanFrame frame{};
				frame.dlc = sizeof(ShirasuMode);
				frame.data[0] = CRSLib::to_underlying(mode);

				{
					std::lock_guard lock{mutex};
					can_mode_pub.publish(frame);
				}
			}
		
		private:
			void update(const float target)
			{
				adhoc_canplugins_onehalf::CanFrame frame{};
				frame.dlc = sizeof(float);
				adhoc_canplugins_onehalf::pack(frame.data.data(), target);
				can_target_pub.publish(frame);
			}

			void update_status(const adhoc_canplugins_onehalf::CanFrame::ConstPtr& frame)
			{
				std::lock_guard lock{mutex};

				mode = static_cast<ShirasuMode>(frame->data[0]);
				if(is_ajusting_zero)
				{
					if(mode == ShirasuMode::homing)
					{
						is_entered_homing = true;
					}
					else if(is_entered_homing)
					{
						is_ajusting_zero = false;
						is_entered_homing = false;
					}
				}
			}
		};
	}

	class ShirasuMd final
	{
		std::unique_ptr<Implement::ShirasuMdInner> inner;

	public:
		ShirasuMd(const u32 base_id, ros::NodeHandle& nh):
			inner{std::make_unique<Implement::ShirasuMdInner>(base_id, nh)}
		{}

		ShirasuMd(ShirasuMd&&) = default;
		ShirasuMd& operator=(ShirasuMd&&) = default;

		void velocity_update(const float target)
		{
			inner->velocity_update(target);
		}

		float get_velocity()
		{
			return inner->get_velocity();
		}

		void position_update(const float target)
		{
			inner->position_update(target);
		}

		float get_position()
		{
			return inner->get_position();
		}

		void ajust_zero_point()
		{
			inner->ajust_zero_point();
		}

		bool finished_ajusting_zero()
		{
			return inner->finished_ajusting_zero();
		}

		MotorDriver::Mode get_mode() const noexcept
		{
			return inner->get_mode();
		}

		void to_stop()
		{
			inner->to_stop();
		}

		[[deprecated]] void to_powerless()
		{
			inner->to_powerless();
		}

		void change_mode(const ShirasuMode mode)
		{
			inner->change_mode(mode);
		}
	};

	static_assert(MotorDriver<ShirasuMd>);
	static_assert(VelocityControlable<ShirasuMd>);
	static_assert(PositionControlable<ShirasuMd>);
}