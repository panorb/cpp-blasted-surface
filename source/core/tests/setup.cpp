#include <catch2/reporters/catch_reporter_event_listener.hpp>
#include <catch2/reporters/catch_reporter_registrars.hpp>
#include <spdlog/cfg/env.h>

class Test_run_listener final : public Catch::EventListenerBase
{
public:
	using EventListenerBase::EventListenerBase;

	void testRunStarting(Catch::TestRunInfo const& testRunInfo) override
	{
		spdlog::cfg::load_env_levels();
	}
	
};

CATCH_REGISTER_LISTENER(Test_run_listener);