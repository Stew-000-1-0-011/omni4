#pragma once

#if (__cplusplus >= 202002L)
#include <concepts>

namespace crs_lib::MotorDriver
{
	enum class Mode
	{
		run,
		stop,
		powerless,

		error
	};

	template<class T>
	concept MotorDriver = requires(T motor_driver, const T const_motor_driver, const float target)
	{

		requires std::move_constructible<T>;
		{const_motor_driver.get_mode()} noexcept -> std::same_as<Mode>;
		motor_driver.to_stop();
		motor_driver.to_powerless();
	};

	template<class T>
	concept VelocityControlable = MotorDriver<T> && requires(T motor_driver, const float target)
	{
		motor_driver.velocity_update(target);
		{motor_driver.get_velocity()} -> std::same_as<float>;
	};

	template<class T>
	concept PositionControlable = MotorDriver<T> && requires(T motor_driver, const float target)
	{
		motor_driver.position_update(target);
		{motor_driver.get_position()} -> std::same_as<float>;
		{motor_driver.ajust_zero_point()};
		{motor_driver.finished_ajusting_zero()} -> std::same_as<bool>;
	};
}

#else
#include <type_traits>

namespace crs_lib::MotorDriver
{
	enum class Mode
	{
		run,
		stop,
		powerless,

		error
	};

	template<class T, class U>
	concept same_as = std::is_same_v<T, U>;

	template<class T>
	concept MotorDriver = requires(T motor_driver, const T const_motor_driver, const float target)
	{

		requires std::is_move_constructible_v<T>;
		{const_motor_driver.get_mode()} noexcept -> same_as<Mode>;
		motor_driver.to_stop();
		motor_driver.to_powerless();
	};

	template<class T>
	concept VelocityControlable = MotorDriver<T> && requires(T motor_driver, const float target)
	{
		motor_driver.velocity_update(target);
		{motor_driver.get_velocity()} -> same_as<float>;
	};

	template<class T>
	concept PositionControlable = MotorDriver<T> && requires(T motor_driver, const float target)
	{
		motor_driver.position_update(target);
		{motor_driver.get_position()} -> same_as<float>;
		{motor_driver.ajust_zero_point()};
		{motor_driver.finished_ajusting_zero()} -> same_as<bool>;
	};
}
#endif
