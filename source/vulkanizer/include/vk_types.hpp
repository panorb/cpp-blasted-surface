#pragma once

#include <vk_mem_alloc.h>
#include <vulkan/vulkan_core.h>

struct Allocated_buffer
{
	VkBuffer buffer;
	VmaAllocation allocation;
	VmaAllocationInfo info;
};
