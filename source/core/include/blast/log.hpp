#pragma once
#include <spdlog/spdlog.h>
#include <spdlog/cfg/env.h>

namespace blast::log {
	inline void init()
	{
		spdlog::cfg::load_env_levels();
	}

	template<typename T>
	void info(const T& msg)
	{
		spdlog::info(msg);
	}

	template<typename T>
	void warn(const T& msg) {
		spdlog::warn(msg);
	}

	template<typename T>
	void error(const T& msg)
	{
		spdlog::error(msg);
	}

	template<typename T>
	void debug(const T& msg)
	{
		spdlog::debug(msg);
	}

	template<typename T>
	void trace(const T& msg)
	{
		spdlog::trace(msg);
	}
}


