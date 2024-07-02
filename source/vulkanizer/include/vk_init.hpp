#pragma once
#include <optional>
#include <vector>
#include <vulkan/vulkan_core.h>

///////////////////////////
// HELPERS               //
///////////////////////////

struct Queue_family_indices {
	std::optional<uint32_t> graphics_family;
	std::optional<uint32_t> present_family;

	bool is_complete() const {
		return graphics_family.has_value() && present_family.has_value();
	}
};

struct Swap_chain_support_details {
	VkSurfaceCapabilitiesKHR capabilities;
	std::vector<VkSurfaceFormatKHR> formats;
	std::vector<VkPresentModeKHR> present_modes;
};