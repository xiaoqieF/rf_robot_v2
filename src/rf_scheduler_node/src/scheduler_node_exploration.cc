#include "scheduler_node_internal.hpp"

namespace rf_scheduler
{

void SchedulerNode::handleExploreMap()
{
    detail::MappingJob(*this).run();
}

} // namespace rf_scheduler
