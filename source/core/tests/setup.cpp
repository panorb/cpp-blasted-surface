#include <catch2/reporters/catch_reporter_event_listener.hpp>
#include <catch2/reporters/catch_reporter_registrars.hpp>
#include <blast/log.hpp>

class Test_run_listener final : public Catch::EventListenerBase
{
public:
	using EventListenerBase::EventListenerBase;

	void testRunStarting(Catch::TestRunInfo const& testRunInfo) override
	{
		blast::log::init();
	}
	
};

CATCH_REGISTER_LISTENER(Test_run_listener);