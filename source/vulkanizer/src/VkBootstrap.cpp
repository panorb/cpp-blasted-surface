/*
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the “Software”), to deal in the Software without restriction, including without
 * limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
 * LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Copyright © 2020 Charles Giessen (charles@lunarg.com)
 */

#include "VkBootstrap.h"

#include <cstring>

#if defined(_WIN32)
#include <fcntl.h>
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif // _WIN32

#if defined(__linux__) || defined(__APPLE__)
#include <dlfcn.h>
#endif

#include <mutex>
#include <algorithm>

namespace vkb {

namespace detail {

GenericFeaturesPNextNode::GenericFeaturesPNextNode() { memset(fields, UINT8_MAX, sizeof(VkBool32) * field_capacity); }

bool GenericFeaturesPNextNode::match(GenericFeaturesPNextNode const& requested, GenericFeaturesPNextNode const& supported) noexcept {
    assert(requested.sType == supported.sType && "Non-matching sTypes in features nodes!");
    for (uint32_t i = 0; i < field_capacity; i++) {
        if (requested.fields[i] && !supported.fields[i]) return false;
    }
    return true;
}

void GenericFeaturesPNextNode::combine(GenericFeaturesPNextNode const& right) noexcept {
    assert(sType == right.sType && "Non-matching sTypes in features nodes!");
    for (uint32_t i = 0; i < GenericFeaturesPNextNode::field_capacity; i++) {
        fields[i] = fields[i] || right.fields[i];
    }
}

bool GenericFeatureChain::match_all(GenericFeatureChain const& extension_requested) const noexcept {
    // Should only be false if extension_supported was unable to be filled out, due to the
    // physical device not supporting vkGetPhysicalDeviceFeatures2 in any capacity.
    if (extension_requested.nodes.size() != nodes.size()) {
        return false;
    }

    for (size_t i = 0; i < nodes.size() && i < nodes.size(); ++i) {
        if (!GenericFeaturesPNextNode::match(extension_requested.nodes[i], nodes[i])) return false;
    }
    return true;
}

bool GenericFeatureChain::find_and_match(GenericFeatureChain const& extensions_requested) const noexcept {
    for (const auto& requested_extension_node : extensions_requested.nodes) {
        bool found = false;
        for (const auto& supported_node : nodes) {
            if (supported_node.sType == requested_extension_node.sType) {
                found = true;
                if (!GenericFeaturesPNextNode::match(requested_extension_node, supported_node)) return false;
                break;
            }
        }
        if (!found) return false;
    }
    return true;
}

void GenericFeatureChain::chain_up(VkPhysicalDeviceFeatures2& feats2) noexcept {
    detail::GenericFeaturesPNextNode* prev = nullptr;
    for (auto& extension : nodes) {
        if (prev != nullptr) {
            prev->pNext = &extension;
        }
        prev = &extension;
    }
    feats2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
    feats2.pNext = !nodes.empty() ? &nodes.at(0) : nullptr;
}

void GenericFeatureChain::combine(GenericFeatureChain const& right) noexcept {
    for (const auto& right_node : right.nodes) {
        bool already_contained = false;
        for (auto& left_node : nodes) {
            if (left_node.sType == right_node.sType) {
                left_node.combine(right_node);
                already_contained = true;
            }
        }
        if (!already_contained) {
            nodes.push_back(right_node);
        }
    }
}


class VulkanFunctions {
    private:
    std::mutex init_mutex;

#if defined(__linux__) || defined(__APPLE__)
    void* library = nullptr;
#elif defined(_WIN32)
    HMODULE library = nullptr;
#endif

    bool load_vulkan_library() {
        // Can immediately return if it has already been loaded
        if (library) {
            return true;
        }
#if defined(__linux__)
        library = dlopen("libvulkan.so.1", RTLD_NOW | RTLD_LOCAL);
        if (!library) library = dlopen("libvulkan.so", RTLD_NOW | RTLD_LOCAL);
#elif defined(__APPLE__)
        library = dlopen("libvulkan.dylib", RTLD_NOW | RTLD_LOCAL);
        if (!library) library = dlopen("libvulkan.1.dylib", RTLD_NOW | RTLD_LOCAL);
        if (!library) library = dlopen("libMoltenVK.dylib", RTLD_NOW | RTLD_LOCAL);
#elif defined(_WIN32)
        library = LoadLibrary(TEXT("vulkan-1.dll"));
#else
        assert(false && "Unsupported platform");
#endif
        if (!library) return false;
        load_func(ptr_vkGetInstanceProcAddr, "vkGetInstanceProcAddr");
        return ptr_vkGetInstanceProcAddr != nullptr;
    }

    template <typename T> void load_func(T& func_dest, const char* func_name) {
#if defined(__linux__) || defined(__APPLE__)
        func_dest = reinterpret_cast<T>(dlsym(library, func_name));
#elif defined(_WIN32)
        func_dest = reinterpret_cast<T>(GetProcAddress(library, func_name));
#endif
    }
    void close() {
#if defined(__linux__) || defined(__APPLE__)
        dlclose(library);
#elif defined(_WIN32)
        FreeLibrary(library);
#endif
        library = 0;
    }

    public:
    bool init_vulkan_funcs(PFN_vkGetInstanceProcAddr fp_vkGetInstanceProcAddr = nullptr) {
        std::lock_guard<std::mutex> lg(init_mutex);
        if (fp_vkGetInstanceProcAddr != nullptr) {
            ptr_vkGetInstanceProcAddr = fp_vkGetInstanceProcAddr;
        } else {
            bool ret = load_vulkan_library();
            if (!ret) return false;
        }

        fp_vkEnumerateInstanceExtensionProperties = reinterpret_cast<PFN_vkEnumerateInstanceExtensionProperties>(
            ptr_vkGetInstanceProcAddr(VK_NULL_HANDLE, "vkEnumerateInstanceExtensionProperties"));
        fp_vkEnumerateInstanceLayerProperties = reinterpret_cast<PFN_vkEnumerateInstanceLayerProperties>(
            ptr_vkGetInstanceProcAddr(VK_NULL_HANDLE, "vkEnumerateInstanceLayerProperties"));
        fp_vkEnumerateInstanceVersion = reinterpret_cast<PFN_vkEnumerateInstanceVersion>(
            ptr_vkGetInstanceProcAddr(VK_NULL_HANDLE, "vkEnumerateInstanceVersion"));
        fp_vkCreateInstance =
            reinterpret_cast<PFN_vkCreateInstance>(ptr_vkGetInstanceProcAddr(VK_NULL_HANDLE, "vkCreateInstance"));
        return true;
    }

    public:
    template <typename T> void get_inst_proc_addr(T& out_ptr, const char* func_name) {
        out_ptr = reinterpret_cast<T>(ptr_vkGetInstanceProcAddr(instance, func_name));
    }

    template <typename T> void get_device_proc_addr(VkDevice device, T& out_ptr, const char* func_name) {
        out_ptr = reinterpret_cast<T>(fp_vkGetDeviceProcAddr(device, func_name));
    }

    PFN_vkGetInstanceProcAddr ptr_vkGetInstanceProcAddr = nullptr;
    VkInstance instance = nullptr;

    PFN_vkEnumerateInstanceExtensionProperties fp_vkEnumerateInstanceExtensionProperties = nullptr;
    PFN_vkEnumerateInstanceLayerProperties fp_vkEnumerateInstanceLayerProperties = nullptr;
    PFN_vkEnumerateInstanceVersion fp_vkEnumerateInstanceVersion = nullptr;
    PFN_vkCreateInstance fp_vkCreateInstance = nullptr;

    PFN_vkDestroyInstance fp_vkDestroyInstance = nullptr;
    PFN_vkCreateDebugUtilsMessengerEXT fp_vkCreateDebugUtilsMessengerEXT = nullptr;
    PFN_vkDestroyDebugUtilsMessengerEXT fp_vkDestroyDebugUtilsMessengerEXT = nullptr;
    PFN_vkEnumeratePhysicalDevices fp_vkEnumeratePhysicalDevices = nullptr;
    PFN_vkGetPhysicalDeviceFeatures fp_vkGetPhysicalDeviceFeatures = nullptr;
    PFN_vkGetPhysicalDeviceFeatures2 fp_vkGetPhysicalDeviceFeatures2 = nullptr;
    PFN_vkGetPhysicalDeviceFeatures2KHR fp_vkGetPhysicalDeviceFeatures2KHR = nullptr;
    PFN_vkGetPhysicalDeviceProperties fp_vkGetPhysicalDeviceProperties = nullptr;
    PFN_vkGetPhysicalDeviceQueueFamilyProperties fp_vkGetPhysicalDeviceQueueFamilyProperties = nullptr;
    PFN_vkGetPhysicalDeviceMemoryProperties fp_vkGetPhysicalDeviceMemoryProperties = nullptr;
    PFN_vkEnumerateDeviceExtensionProperties fp_vkEnumerateDeviceExtensionProperties = nullptr;

    PFN_vkCreateDevice fp_vkCreateDevice = nullptr;
    PFN_vkGetDeviceProcAddr fp_vkGetDeviceProcAddr = nullptr;

    PFN_vkDestroySurfaceKHR fp_vkDestroySurfaceKHR = nullptr;
    PFN_vkGetPhysicalDeviceSurfaceSupportKHR fp_vkGetPhysicalDeviceSurfaceSupportKHR = nullptr;
    PFN_vkGetPhysicalDeviceSurfaceFormatsKHR fp_vkGetPhysicalDeviceSurfaceFormatsKHR = nullptr;
    PFN_vkGetPhysicalDeviceSurfacePresentModesKHR fp_vkGetPhysicalDeviceSurfacePresentModesKHR = nullptr;
    PFN_vkGetPhysicalDeviceSurfaceCapabilitiesKHR fp_vkGetPhysicalDeviceSurfaceCapabilitiesKHR = nullptr;

    void init_instance_funcs(VkInstance inst) {
        instance = inst;
        get_inst_proc_addr(fp_vkDestroyInstance, "vkDestroyInstance");
        get_inst_proc_addr(fp_vkCreateDebugUtilsMessengerEXT, "vkCreateDebugUtilsMessengerEXT");
        get_inst_proc_addr(fp_vkDestroyDebugUtilsMessengerEXT, "vkDestroyDebugUtilsMessengerEXT");
        get_inst_proc_addr(fp_vkEnumeratePhysicalDevices, "vkEnumeratePhysicalDevices");

        get_inst_proc_addr(fp_vkGetPhysicalDeviceFeatures, "vkGetPhysicalDeviceFeatures");
        get_inst_proc_addr(fp_vkGetPhysicalDeviceFeatures2, "vkGetPhysicalDeviceFeatures2");
        get_inst_proc_addr(fp_vkGetPhysicalDeviceFeatures2KHR, "vkGetPhysicalDeviceFeatures2KHR");
        get_inst_proc_addr(fp_vkGetPhysicalDeviceProperties, "vkGetPhysicalDeviceProperties");
        get_inst_proc_addr(fp_vkGetPhysicalDeviceQueueFamilyProperties, "vkGetPhysicalDeviceQueueFamilyProperties");
        get_inst_proc_addr(fp_vkGetPhysicalDeviceMemoryProperties, "vkGetPhysicalDeviceMemoryProperties");
        get_inst_proc_addr(fp_vkEnumerateDeviceExtensionProperties, "vkEnumerateDeviceExtensionProperties");

        get_inst_proc_addr(fp_vkCreateDevice, "vkCreateDevice");
        get_inst_proc_addr(fp_vkGetDeviceProcAddr, "vkGetDeviceProcAddr");

        get_inst_proc_addr(fp_vkDestroySurfaceKHR, "vkDestroySurfaceKHR");
        get_inst_proc_addr(fp_vkGetPhysicalDeviceSurfaceSupportKHR, "vkGetPhysicalDeviceSurfaceSupportKHR");
        get_inst_proc_addr(fp_vkGetPhysicalDeviceSurfaceFormatsKHR, "vkGetPhysicalDeviceSurfaceFormatsKHR");
        get_inst_proc_addr(fp_vkGetPhysicalDeviceSurfacePresentModesKHR, "vkGetPhysicalDeviceSurfacePresentModesKHR");
        get_inst_proc_addr(fp_vkGetPhysicalDeviceSurfaceCapabilitiesKHR, "vkGetPhysicalDeviceSurfaceCapabilitiesKHR");
    }
};

static VulkanFunctions& vulkan_functions() {
    static VulkanFunctions v;
    return v;
}

// Helper for robustly executing the two-call pattern
template <typename T, typename F, typename... Ts> auto get_vector(std::vector<T>& out, F&& f, Ts&&... ts) -> VkResult {
    uint32_t count = 0;
    VkResult err;
    do {
        err = f(ts..., &count, nullptr);
        if (err != VK_SUCCESS) {
            return err;
        };
        out.resize(count);
        err = f(ts..., &count, out.data());
        out.resize(count);
    } while (err == VK_INCOMPLETE);
    return err;
}

template <typename T, typename F, typename... Ts> auto get_vector_noerror(F&& f, Ts&&... ts) -> std::vector<T> {
    uint32_t count = 0;
    std::vector<T> results;
    f(ts..., &count, nullptr);
    results.resize(count);
    f(ts..., &count, results.data());
    results.resize(count);
    return results;
}
} // namespace detail

const char* to_string_message_severity(VkDebugUtilsMessageSeverityFlagBitsEXT s) {
    switch (s) {
        case VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT:
            return "VERBOSE";
        case VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT:
            return "ERROR";
        case VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT:
            return "WARNING";
        case VK_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT:
            return "INFO";
        default:
            return "UNKNOWN";
    }
}
const char* to_string_message_type(VkDebugUtilsMessageTypeFlagsEXT s) {
    if (s == 7) return "General | Validation | Performance";
    if (s == 6) return "Validation | Performance";
    if (s == 5) return "General | Performance";
    if (s == 4 /*VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT*/) return "Performance";
    if (s == 3) return "General | Validation";
    if (s == 2 /*VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT*/) return "Validation";
    if (s == 1 /*VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT*/) return "General";
    return "Unknown";
}

VkResult create_debug_utils_messenger(VkInstance instance,
    PFN_vkDebugUtilsMessengerCallbackEXT debug_callback,
    VkDebugUtilsMessageSeverityFlagsEXT severity,
    VkDebugUtilsMessageTypeFlagsEXT type,
    void* user_data_pointer,
    VkDebugUtilsMessengerEXT* pDebugMessenger,
    VkAllocationCallbacks* allocation_callbacks) {

    if (debug_callback == nullptr) debug_callback = default_debug_callback;
    VkDebugUtilsMessengerCreateInfoEXT messengerCreateInfo = {};
    messengerCreateInfo.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
    messengerCreateInfo.pNext = nullptr;
    messengerCreateInfo.messageSeverity = severity;
    messengerCreateInfo.messageType = type;
    messengerCreateInfo.pfnUserCallback = debug_callback;
    messengerCreateInfo.pUserData = user_data_pointer;

    if (detail::vulkan_functions().fp_vkCreateDebugUtilsMessengerEXT != nullptr) {
        return detail::vulkan_functions().fp_vkCreateDebugUtilsMessengerEXT(
            instance, &messengerCreateInfo, allocation_callbacks, pDebugMessenger);
    } else {
        return VK_ERROR_EXTENSION_NOT_PRESENT;
    }
}

void destroy_debug_utils_messenger(
    VkInstance instance, VkDebugUtilsMessengerEXT debugMessenger, VkAllocationCallbacks* allocation_callbacks) {

    if (detail::vulkan_functions().fp_vkDestroyDebugUtilsMessengerEXT != nullptr) {
        detail::vulkan_functions().fp_vkDestroyDebugUtilsMessengerEXT(instance, debugMessenger, allocation_callbacks);
    }
}

namespace detail {
bool check_layer_supported(std::vector<VkLayerProperties> const& available_layers, const char* layer_name) {
    if (!layer_name) return false;
    for (const auto& layer_properties : available_layers) {
        if (strcmp(layer_name, layer_properties.layerName) == 0) {
            return true;
        }
    }
    return false;
}

bool check_layers_supported(std::vector<VkLayerProperties> const& available_layers, std::vector<const char*> const& layer_names) {
    bool all_found = true;
    for (const auto& layer_name : layer_names) {
        bool found = check_layer_supported(available_layers, layer_name);
        if (!found) all_found = false;
    }
    return all_found;
}

bool check_extension_supported(std::vector<VkExtensionProperties> const& available_extensions, const char* extension_name) {
    if (!extension_name) return false;
    for (const auto& extension_properties : available_extensions) {
        if (strcmp(extension_name, extension_properties.extensionName) == 0) {
            return true;
        }
    }
    return false;
}

bool check_extensions_supported(
    std::vector<VkExtensionProperties> const& available_extensions, std::vector<const char*> const& extension_names) {
    bool all_found = true;
    for (const auto& extension_name : extension_names) {
        bool found = check_extension_supported(available_extensions, extension_name);
        if (!found) all_found = false;
    }
    return all_found;
}

template <typename T> void setup_pNext_chain(T& structure, std::vector<VkBaseOutStructure*> const& structs) {
    structure.pNext = nullptr;
    if (structs.size() <= 0) return;
    for (size_t i = 0; i < structs.size() - 1; i++) {
        structs.at(i)->pNext = structs.at(i + 1);
    }
    structure.pNext = structs.at(0);
}
const char* validation_layer_name = "VK_LAYER_KHRONOS_validation";

struct InstanceErrorCategory : std::error_category {
    const char* name() const noexcept override { return "vkb_instance"; }
    std::string message(int err) const override { return to_string(static_cast<InstanceError>(err)); }
};
const InstanceErrorCategory instance_error_category;

struct PhysicalDeviceErrorCategory : std::error_category {
    const char* name() const noexcept override { return "vkb_physical_device"; }
    std::string message(int err) const override { return to_string(static_cast<PhysicalDeviceError>(err)); }
};
const PhysicalDeviceErrorCategory physical_device_error_category;

struct QueueErrorCategory : std::error_category {
    const char* name() const noexcept override { return "vkb_queue"; }
    std::string message(int err) const override { return to_string(static_cast<QueueError>(err)); }
};
const QueueErrorCategory queue_error_category;

struct DeviceErrorCategory : std::error_category {
    const char* name() const noexcept override { return "vkb_device"; }
    std::string message(int err) const override { return to_string(static_cast<DeviceError>(err)); }
};
const DeviceErrorCategory device_error_category;

struct SwapchainErrorCategory : std::error_category {
    const char* name() const noexcept override { return "vbk_swapchain"; }
    std::string message(int err) const override { return to_string(static_cast<SwapchainError>(err)); }
};
const SwapchainErrorCategory swapchain_error_category;

} // namespace detail

std::error_code make_error_code(InstanceError instance_error) {
    return { static_cast<int>(instance_error), detail::instance_error_category };
}
std::error_code make_error_code(PhysicalDeviceError physical_device_error) {
    return { static_cast<int>(physical_device_error), detail::physical_device_error_category };
}
std::error_code make_error_code(QueueError queue_error) {
    return { static_cast<int>(queue_error), detail::queue_error_category };
}
std::error_code make_error_code(DeviceError device_error) {
    return { static_cast<int>(device_error), detail::device_error_category };
}
std::error_code make_error_code(SwapchainError swapchain_error) {
    return { static_cast<int>(swapchain_error), detail::swapchain_error_category };
}
#define CASE_TO_STRING(CATEGORY, TYPE)                                                                                 \
    case CATEGORY::TYPE:                                                                                               \
        return #TYPE;

const char* to_string(InstanceError err) {
    switch (err) {
        CASE_TO_STRING(InstanceError, vulkan_unavailable)
        CASE_TO_STRING(InstanceError, vulkan_version_unavailable)
        CASE_TO_STRING(InstanceError, vulkan_version_1_1_unavailable)
        CASE_TO_STRING(InstanceError, vulkan_version_1_2_unavailable)
        CASE_TO_STRING(InstanceError, failed_create_debug_messenger)
        CASE_TO_STRING(InstanceError, failed_create_instance)
        CASE_TO_STRING(InstanceError, requested_layers_not_present)
        CASE_TO_STRING(InstanceError, requested_extensions_not_present)
        CASE_TO_STRING(InstanceError, windowing_extensions_not_present)
        default:
            return "";
    }
}
const char* to_string(PhysicalDeviceError err) {
    switch (err) {
        CASE_TO_STRING(PhysicalDeviceError, no_surface_provided)
        CASE_TO_STRING(PhysicalDeviceError, failed_enumerate_physical_devices)
        CASE_TO_STRING(PhysicalDeviceError, no_physical_devices_found)
        CASE_TO_STRING(PhysicalDeviceError, no_suitable_device)
        default:
            return "";
    }
}
const char* to_string(QueueError err) {
    switch (err) {
        CASE_TO_STRING(QueueError, present_unavailable)
        CASE_TO_STRING(QueueError, graphics_unavailable)
        CASE_TO_STRING(QueueError, compute_unavailable)
        CASE_TO_STRING(QueueError, transfer_unavailable)
        CASE_TO_STRING(QueueError, queue_index_out_of_range)
        CASE_TO_STRING(QueueError, invalid_queue_family_index)
        default:
            return "";
    }
}
const char* to_string(DeviceError err) {
    switch (err) {
        CASE_TO_STRING(DeviceError, failed_create_device)
        default:
            return "";
    }
}
const char* to_string(SwapchainError err) {
    switch (err) {
        CASE_TO_STRING(SwapchainError, surface_handle_not_provided)
        CASE_TO_STRING(SwapchainError, failed_query_surface_support_details)
        CASE_TO_STRING(SwapchainError, failed_create_swapchain)
        CASE_TO_STRING(SwapchainError, failed_get_swapchain_images)
        CASE_TO_STRING(SwapchainError, failed_create_swapchain_image_views)
        CASE_TO_STRING(SwapchainError, required_min_image_count_too_low)
        CASE_TO_STRING(SwapchainError, required_usage_not_supported)
        default:
            return "";
    }
}

Result<SystemInfo> SystemInfo::get_system_info() {
    if (!detail::vulkan_functions().init_vulkan_funcs(nullptr)) {
        return make_error_code(InstanceError::vulkan_unavailable);
    }
    return SystemInfo();
}

Result<SystemInfo> SystemInfo::get_system_info(PFN_vkGetInstanceProcAddr fp_vkGetInstanceProcAddr) {
    // Using externally provided function pointers, assume the loader is available
    if (!detail::vulkan_functions().init_vulkan_funcs(fp_vkGetInstanceProcAddr)) {
        return make_error_code(InstanceError::vulkan_unavailable);
    }
    return SystemInfo();
}

SystemInfo::SystemInfo() {
    auto available_layers_ret = detail::get_vector<VkLayerProperties>(
        this->available_layers, detail::vulkan_functions().fp_vkEnumerateInstanceLayerProperties);
    if (available_layers_ret != VK_SUCCESS) {
        this->available_layers.clear();
    }

    for (auto& layer : this->available_layers)
        if (strcmp(layer.layerName, detail::validation_layer_name) == 0) validation_layers_available = true;

    auto available_extensions_ret = detail::get_vector<VkExtensionProperties>(
        this->available_extensions, detail::vulkan_functions().fp_vkEnumerateInstanceExtensionProperties, nullptr);
    if (available_extensions_ret != VK_SUCCESS) {
        this->available_extensions.clear();
    }

    for (auto& ext : this->available_extensions) {
        if (strcmp(ext.extensionName, VK_EXT_DEBUG_UTILS_EXTENSION_NAME) == 0) {
            debug_utils_available = true;
        }
    }

    for (auto& layer : this->available_layers) {
        std::vector<VkExtensionProperties> layer_extensions;
        auto layer_extensions_ret = detail::get_vector<VkExtensionProperties>(
            layer_extensions, detail::vulkan_functions().fp_vkEnumerateInstanceExtensionProperties, layer.layerName);
        if (layer_extensions_ret == VK_SUCCESS) {
            this->available_extensions.insert(
                this->available_extensions.end(), layer_extensions.begin(), layer_extensions.end());
            for (auto& ext : layer_extensions) {
                if (strcmp(ext.extensionName, VK_EXT_DEBUG_UTILS_EXTENSION_NAME) == 0) {
                    debug_utils_available = true;
                }
            }
        }
    }
}
bool SystemInfo::is_extension_available(const char* extension_name) const {
    if (!extension_name) return false;
    return detail::check_extension_supported(available_extensions, extension_name);
}
bool SystemInfo::is_layer_available(const char* layer_name) const {
    if (!layer_name) return false;
    return detail::check_layer_supported(available_layers, layer_name);
}
void destroy_surface(Instance const& instance, VkSurfaceKHR surface) {
    if (instance.instance != VK_NULL_HANDLE && surface != VK_NULL_HANDLE) {
        detail::vulkan_functions().fp_vkDestroySurfaceKHR(instance.instance, surface, instance.allocation_callbacks);
    }
}
void destroy_surface(VkInstance instance, VkSurfaceKHR surface, VkAllocationCallbacks* callbacks) {
    if (instance != VK_NULL_HANDLE && surface != VK_NULL_HANDLE) {
        detail::vulkan_functions().fp_vkDestroySurfaceKHR(instance, surface, callbacks);
    }
}
void destroy_instance(Instance const& instance) {
    if (instance.instance != VK_NULL_HANDLE) {
        if (instance.debug_messenger != VK_NULL_HANDLE)
            destroy_debug_utils_messenger(instance.instance, instance.debug_messenger, instance.allocation_callbacks);
        detail::vulkan_functions().fp_vkDestroyInstance(instance.instance, instance.allocation_callbacks);
    }
}

Instance::operator VkInstance() const { return this->instance; }

InstanceDispatchTable Instance::make_table() const { return { instance, fp_vkGetInstanceProcAddr }; }

InstanceBuilder::InstanceBuilder(PFN_vkGetInstanceProcAddr fp_vkGetInstanceProcAddr) {
    info.fp_vkGetInstanceProcAddr = fp_vkGetInstanceProcAddr;
}
InstanceBuilder::InstanceBuilder() {}

Result<Instance> InstanceBuilder::build() const {

    auto sys_info_ret = SystemInfo::get_system_info(info.fp_vkGetInstanceProcAddr);
    if (!sys_info_ret) return sys_info_ret.error();
    auto system = sys_info_ret.value();

    uint32_t instance_version = VKB_VK_API_VERSION_1_0;

    if (info.minimum_instance_version > VKB_VK_API_VERSION_1_0 || info.required_api_version > VKB_VK_API_VERSION_1_0 ||
        info.desired_api_version > VKB_VK_API_VERSION_1_0) {
        PFN_vkEnumerateInstanceVersion pfn_vkEnumerateInstanceVersion = detail::vulkan_functions().fp_vkEnumerateInstanceVersion;

        if (pfn_vkEnumerateInstanceVersion != nullptr) {
            VkResult res = pfn_vkEnumerateInstanceVersion(&instance_version);
            // Should always return VK_SUCCESS
            if (res != VK_SUCCESS && info.required_api_version > 0)
                return make_error_code(InstanceError::vulkan_version_unavailable);
        }
        if (pfn_vkEnumerateInstanceVersion == nullptr || instance_version < info.minimum_instance_version ||
            (info.minimum_instance_version == 0 && instance_version < info.required_api_version)) {
            if (VK_VERSION_MINOR(info.required_api_version) == 2)
                return make_error_code(InstanceError::vulkan_version_1_2_unavailable);
            else if (VK_VERSION_MINOR(info.required_api_version))
                return make_error_code(InstanceError::vulkan_version_1_1_unavailable);
            else
                return make_error_code(InstanceError::vulkan_version_unavailable);
        }
    }

    uint32_t api_version = instance_version < VKB_VK_API_VERSION_1_1 ? instance_version : info.required_api_version;

    if (info.desired_api_version > VKB_VK_API_VERSION_1_0 && instance_version >= info.desired_api_version) {
        instance_version = info.desired_api_version;
        api_version = info.desired_api_version;
    }

    VkApplicationInfo app_info = {};
    app_info.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    app_info.pNext = nullptr;
    app_info.pApplicationName = info.app_name != nullptr ? info.app_name : "";
    app_info.applicationVersion = info.application_version;
    app_info.pEngineName = info.engine_name != nullptr ? info.engine_name : "";
    app_info.engineVersion = info.engine_version;
    app_info.apiVersion = api_version;

    std::vector<const char*> extensions;
    std::vector<const char*> layers;

    for (auto& ext : info.extensions)
        extensions.push_back(ext);
    if (info.debug_callback != nullptr && info.use_debug_messenger && system.debug_utils_available) {
        extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
    }
    bool properties2_ext_enabled =
        api_version < VKB_VK_API_VERSION_1_1 && detail::check_extension_supported(system.available_extensions,
                                                    VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME);
    if (properties2_ext_enabled) {
        extensions.push_back(VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME);
    }

#if defined(VK_KHR_portability_enumeration)
    bool portability_enumeration_support =
        detail::check_extension_supported(system.available_extensions, VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME);
    if (portability_enumeration_support) {
        extensions.push_back(VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME);
    }
#else
    bool portability_enumeration_support = false;
#endif
    if (!info.headless_context) {
        auto check_add_window_ext = [&](const char* name) -> bool {
            if (!detail::check_extension_supported(system.available_extensions, name)) return false;
            extensions.push_back(name);
            return true;
        };
        bool khr_surface_added = check_add_window_ext("VK_KHR_surface");
#if defined(_WIN32)
        bool added_window_exts = check_add_window_ext("VK_KHR_win32_surface");
#elif defined(__ANDROID__)
        bool added_window_exts = check_add_window_ext("VK_KHR_android_surface");
#elif defined(_DIRECT2DISPLAY)
        bool added_window_exts = check_add_window_ext("VK_KHR_display");
#elif defined(__linux__)
        // make sure all three calls to check_add_window_ext, don't allow short circuiting
        bool added_window_exts = check_add_window_ext("VK_KHR_xcb_surface");
        added_window_exts = check_add_window_ext("VK_KHR_xlib_surface") || added_window_exts;
        added_window_exts = check_add_window_ext("VK_KHR_wayland_surface") || added_window_exts;
#elif defined(__APPLE__)
        bool added_window_exts = check_add_window_ext("VK_EXT_metal_surface");
#endif
        if (!khr_surface_added || !added_window_exts)
            return make_error_code(InstanceError::windowing_extensions_not_present);
    }
    bool all_extensions_supported = detail::check_extensions_supported(system.available_extensions, extensions);
    if (!all_extensions_supported) {
        return make_error_code(InstanceError::requested_extensions_not_present);
    }

    for (auto& layer : info.layers)
        layers.push_back(layer);

    if (info.enable_validation_layers || (info.request_validation_layers && system.validation_layers_available)) {
        layers.push_back(detail::validation_layer_name);
    }
    bool all_layers_supported = detail::check_layers_supported(system.available_layers, layers);
    if (!all_layers_supported) {
        return make_error_code(InstanceError::requested_layers_not_present);
    }

    std::vector<VkBaseOutStructure*> pNext_chain;

    VkDebugUtilsMessengerCreateInfoEXT messengerCreateInfo = {};
    if (info.use_debug_messenger) {
        messengerCreateInfo.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
        messengerCreateInfo.pNext = nullptr;
        messengerCreateInfo.messageSeverity = info.debug_message_severity;
        messengerCreateInfo.messageType = info.debug_message_type;
        messengerCreateInfo.pfnUserCallback = info.debug_callback;
        messengerCreateInfo.pUserData = info.debug_user_data_pointer;
        pNext_chain.push_back(reinterpret_cast<VkBaseOutStructure*>(&messengerCreateInfo));
    }

    VkValidationFeaturesEXT features{};
    if (info.enabled_validation_features.size() != 0 || info.disabled_validation_features.size()) {
        features.sType = VK_STRUCTURE_TYPE_VALIDATION_FEATURES_EXT;
        features.pNext = nullptr;
        features.enabledValidationFeatureCount = static_cast<uint32_t>(info.enabled_validation_features.size());
        features.pEnabledValidationFeatures = info.enabled_validation_features.data();
        features.disabledValidationFeatureCount = static_cast<uint32_t>(info.disabled_validation_features.size());
        features.pDisabledValidationFeatures = info.disabled_validation_features.data();
        pNext_chain.push_back(reinterpret_cast<VkBaseOutStructure*>(&features));
    }

    VkValidationFlagsEXT checks{};
    if (info.disabled_validation_checks.size() != 0) {
        checks.sType = VK_STRUCTURE_TYPE_VALIDATION_FLAGS_EXT;
        checks.pNext = nullptr;
        checks.disabledValidationCheckCount = static_cast<uint32_t>(info.disabled_validation_checks.size());
        checks.pDisabledValidationChecks = info.disabled_validation_checks.data();
        pNext_chain.push_back(reinterpret_cast<VkBaseOutStructure*>(&checks));
    }

    VkInstanceCreateInfo instance_create_info = {};
    instance_create_info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    detail::setup_pNext_chain(instance_create_info, pNext_chain);
#if !defined(NDEBUG)
    for (auto& node : pNext_chain) {
        assert(node->sType != VK_STRUCTURE_TYPE_APPLICATION_INFO);
    }
#endif
    instance_create_info.flags = info.flags;
    instance_create_info.pApplicationInfo = &app_info;
    instance_create_info.enabledExtensionCount = static_cast<uint32_t>(extensions.size());
    instance_create_info.ppEnabledExtensionNames = extensions.data();
    instance_create_info.enabledLayerCount = static_cast<uint32_t>(layers.size());
    instance_create_info.ppEnabledLayerNames = layers.data();
#if defined(VK_KHR_portability_enumeration)
    if (portability_enumeration_support) {
        instance_create_info.flags |= VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR;
    }
#endif

    Instance instance;
    VkResult res =
        detail::vulkan_functions().fp_vkCreateInstance(&instance_create_info, info.allocation_callbacks, &instance.instance);
    if (res != VK_SUCCESS) return Result<Instance>(InstanceError::failed_create_instance, res);

    detail::vulkan_functions().init_instance_funcs(instance.instance);

    if (info.use_debug_messenger) {
        res = create_debug_utils_messenger(instance.instance,
            info.debug_callback,
            info.debug_message_severity,
            info.debug_message_type,
            info.debug_user_data_pointer,
            &instance.debug_messenger,
            info.allocation_callbacks);
        if (res != VK_SUCCESS) {
            return Result<Instance>(InstanceError::failed_create_debug_messenger, res);
        }
    }

    instance.headless = info.headless_context;
    instance.properties2_ext_enabled = properties2_ext_enabled;
    instance.allocation_callbacks = info.allocation_callbacks;
    instance.instance_version = instance_version;
    instance.api_version = api_version;
    instance.fp_vkGetInstanceProcAddr = detail::vulkan_functions().ptr_vkGetInstanceProcAddr;
    instance.fp_vkGetDeviceProcAddr = detail::vulkan_functions().fp_vkGetDeviceProcAddr;
    return instance;
}

InstanceBuilder& InstanceBuilder::set_app_name(const char* app_name) {
    if (!app_name) return *this;
    info.app_name = app_name;
    return *this;
}
InstanceBuilder& InstanceBuilder::set_engine_name(const char* engine_name) {
    if (!engine_name) return *this;
    info.engine_name = engine_name;
    return *this;
}
InstanceBuilder& InstanceBuilder::set_app_version(uint32_t app_version) {
    info.application_version = app_version;
    return *this;
}
InstanceBuilder& InstanceBuilder::set_app_version(uint32_t major, uint32_t minor, uint32_t patch) {
    info.application_version = VKB_MAKE_VK_VERSION(0, major, minor, patch);
    return *this;
}
InstanceBuilder& InstanceBuilder::set_engine_version(uint32_t engine_version) {
    info.engine_version = engine_version;
    return *this;
}
InstanceBuilder& InstanceBuilder::set_engine_version(uint32_t major, uint32_t minor, uint32_t patch) {
    info.engine_version = VKB_MAKE_VK_VERSION(0, major, minor, patch);
    return *this;
}
InstanceBuilder& InstanceBuilder::require_api_version(uint32_t required_api_version) {
    info.required_api_version = required_api_version;
    return *this;
}
InstanceBuilder& InstanceBuilder::require_api_version(uint32_t major, uint32_t minor, uint32_t patch) {
    info.required_api_version = VKB_MAKE_VK_VERSION(0, major, minor, patch);
    return *this;
}
InstanceBuilder& InstanceBuilder::set_minimum_instance_version(uint32_t minimum_instance_version) {
    info.minimum_instance_version = minimum_instance_version;
    return *this;
}
InstanceBuilder& InstanceBuilder::set_minimum_instance_version(uint32_t major, uint32_t minor, uint32_t patch) {
    info.minimum_instance_version = VKB_MAKE_VK_VERSION(0, major, minor, patch);
    return *this;
}
InstanceBuilder& InstanceBuilder::desire_api_version(uint32_t preferred_vulkan_version) {
    info.desired_api_version = preferred_vulkan_version;
    return *this;
}
InstanceBuilder& InstanceBuilder::desire_api_version(uint32_t major, uint32_t minor, uint32_t patch) {
    info.desired_api_version = VKB_MAKE_VK_VERSION(0, major, minor, patch);
    return *this;
}
InstanceBuilder& InstanceBuilder::enable_layer(const char* layer_name) {
    if (!layer_name) return *this;
    info.layers.push_back(layer_name);
    return *this;
}
InstanceBuilder& InstanceBuilder::enable_extension(const char* extension_name) {
    if (!extension_name) return *this;
    info.extensions.push_back(extension_name);
    return *this;
}
InstanceBuilder& InstanceBuilder::enable_extensions(std::vector<const char*> const& extensions) {
    for (const auto extension : extensions) {
        info.extensions.push_back(extension);
    }
    return *this;
}
InstanceBuilder& InstanceBuilder::enable_extensions(size_t count, const char* const* extensions) {
    if (!extensions || count == 0) return *this;
    for (size_t i = 0; i < count; i++) {
        info.extensions.push_back(extensions[i]);
    }
    return *this;
}
InstanceBuilder& InstanceBuilder::enable_validation_layers(bool enable_validation) {
    info.enable_validation_layers = enable_validation;
    return *this;
}
InstanceBuilder& InstanceBuilder::request_validation_layers(bool enable_validation) {
    info.request_validation_layers = enable_validation;
    return *this;
}

InstanceBuilder& InstanceBuilder::use_default_debug_messenger() {
    info.use_debug_messenger = true;
    info.debug_callback = default_debug_callback;
    return *this;
}
InstanceBuilder& InstanceBuilder::set_debug_callback(PFN_vkDebugUtilsMessengerCallbackEXT callback) {
    info.use_debug_messenger = true;
    info.debug_callback = callback;
    return *this;
}
InstanceBuilder& InstanceBuilder::set_debug_callback_user_data_pointer(void* user_data_pointer) {
    info.debug_user_data_pointer = user_data_pointer;
    return *this;
}
InstanceBuilder& InstanceBuilder::set_headless(bool headless) {
    info.headless_context = headless;
    return *this;
}
InstanceBuilder& InstanceBuilder::set_debug_messenger_severity(VkDebugUtilsMessageSeverityFlagsEXT severity) {
    info.debug_message_severity = severity;
    return *this;
}
InstanceBuilder& InstanceBuilder::add_debug_messenger_severity(VkDebugUtilsMessageSeverityFlagsEXT severity) {
    info.debug_message_severity = info.debug_message_severity | severity;
    return *this;
}
InstanceBuilder& InstanceBuilder::set_debug_messenger_type(VkDebugUtilsMessageTypeFlagsEXT type) {
    info.debug_message_type = type;
    return *this;
}
InstanceBuilder& InstanceBuilder::add_debug_messenger_type(VkDebugUtilsMessageTypeFlagsEXT type) {
    info.debug_message_type = info.debug_message_type | type;
    return *this;
}
InstanceBuilder& InstanceBuilder::add_validation_disable(VkValidationCheckEXT check) {
    info.disabled_validation_checks.push_back(check);
    return *this;
}
InstanceBuilder& InstanceBuilder::add_validation_feature_enable(VkValidationFeatureEnableEXT enable) {
    info.enabled_validation_features.push_back(enable);
    return *this;
}
InstanceBuilder& InstanceBuilder::add_validation_feature_disable(VkValidationFeatureDisableEXT disable) {
    info.disabled_validation_features.push_back(disable);
    return *this;
}
InstanceBuilder& InstanceBuilder::set_allocation_callbacks(VkAllocationCallbacks* callbacks) {
    info.allocation_callbacks = callbacks;
    return *this;
}

void destroy_debug_messenger(VkInstance const instance, VkDebugUtilsMessengerEXT const messenger);


// ---- Physical Device ---- //

namespace detail {

std::vector<std::string> check_device_extension_support(
    std::vector<std::string> const& available_extensions, std::vector<std::string> const& desired_extensions) {
    std::vector<std::string> extensions_to_enable;
    for (const auto& avail_ext : available_extensions) {
        for (auto& req_ext : desired_extensions) {
            if (avail_ext == req_ext) {
                extensions_to_enable.push_back(req_ext);
                break;
            }
        }
    }
    return extensions_to_enable;
}

// clang-format off
void combine_features(VkPhysicalDeviceFeatures& dest, VkPhysicalDeviceFeatures src){
    dest.robustBufferAccess = dest.robustBufferAccess || src.robustBufferAccess;
	dest.fullDrawIndexUint32 = dest.fullDrawIndexUint32 || src.fullDrawIndexUint32;
	dest.imageCubeArray = dest.imageCubeArray || src.imageCubeArray;
	dest.independentBlend = dest.independentBlend || src.independentBlend;
	dest.geometryShader = dest.geometryShader || src.geometryShader;
	dest.tessellationShader = dest.tessellationShader || src.tessellationShader;
	dest.sampleRateShading = dest.sampleRateShading || src.sampleRateShading;
	dest.dualSrcBlend = dest.dualSrcBlend || src.dualSrcBlend;
	dest.logicOp = dest.logicOp || src.logicOp;
	dest.multiDrawIndirect = dest.multiDrawIndirect || src.multiDrawIndirect;
	dest.drawIndirectFirstInstance = dest.drawIndirectFirstInstance || src.drawIndirectFirstInstance;
	dest.depthClamp = dest.depthClamp || src.depthClamp;
	dest.depthBiasClamp = dest.depthBiasClamp || src.depthBiasClamp;
	dest.fillModeNonSolid = dest.fillModeNonSolid || src.fillModeNonSolid;
	dest.depthBounds = dest.depthBounds || src.depthBounds;
	dest.wideLines = dest.wideLines || src.wideLines;
	dest.largePoints = dest.largePoints || src.largePoints;
	dest.alphaToOne = dest.alphaToOne || src.alphaToOne;
	dest.multiViewport = dest.multiViewport || src.multiViewport;
	dest.samplerAnisotropy = dest.samplerAnisotropy || src.samplerAnisotropy;
	dest.textureCompressionETC2 = dest.textureCompressionETC2 || src.textureCompressionETC2;
	dest.textureCompressionASTC_LDR = dest.textureCompressionASTC_LDR || src.textureCompressionASTC_LDR;
	dest.textureCompressionBC = dest.textureCompressionBC || src.textureCompressionBC;
	dest.occlusionQueryPrecise = dest.occlusionQueryPrecise || src.occlusionQueryPrecise;
	dest.pipelineStatisticsQuery = dest.pipelineStatisticsQuery || src.pipelineStatisticsQuery;
	dest.vertexPipelineStoresAndAtomics = dest.vertexPipelineStoresAndAtomics || src.vertexPipelineStoresAndAtomics;
	dest.fragmentStoresAndAtomics = dest.fragmentStoresAndAtomics || src.fragmentStoresAndAtomics;
	dest.shaderTessellationAndGeometryPointSize = dest.shaderTessellationAndGeometryPointSize || src.shaderTessellationAndGeometryPointSize;
	dest.shaderImageGatherExtended = dest.shaderImageGatherExtended || src.shaderImageGatherExtended;
	dest.shaderStorageImageExtendedFormats = dest.shaderStorageImageExtendedFormats || src.shaderStorageImageExtendedFormats;
	dest.shaderStorageImageMultisample = dest.shaderStorageImageMultisample || src.shaderStorageImageMultisample;
	dest.shaderStorageImageReadWithoutFormat = dest.shaderStorageImageReadWithoutFormat || src.shaderStorageImageReadWithoutFormat;
	dest.shaderStorageImageWriteWithoutFormat = dest.shaderStorageImageWriteWithoutFormat || src.shaderStorageImageWriteWithoutFormat;
	dest.shaderUniformBufferArrayDynamicIndexing = dest.shaderUniformBufferArrayDynamicIndexing || src.shaderUniformBufferArrayDynamicIndexing;
	dest.shaderSampledImageArrayDynamicIndexing = dest.shaderSampledImageArrayDynamicIndexing || src.shaderSampledImageArrayDynamicIndexing;
	dest.shaderStorageBufferArrayDynamicIndexing = dest.shaderStorageBufferArrayDynamicIndexing || src.shaderStorageBufferArrayDynamicIndexing;
	dest.shaderStorageImageArrayDynamicIndexing = dest.shaderStorageImageArrayDynamicIndexing || src.shaderStorageImageArrayDynamicIndexing;
	dest.shaderClipDistance = dest.shaderClipDistance || src.shaderClipDistance;
	dest.shaderCullDistance = dest.shaderCullDistance || src.shaderCullDistance;
	dest.shaderFloat64 = dest.shaderFloat64 || src.shaderFloat64;
	dest.shaderInt64 = dest.shaderInt64 || src.shaderInt64;
	dest.shaderInt16 = dest.shaderInt16 || src.shaderInt16;
	dest.shaderResourceResidency = dest.shaderResourceResidency || src.shaderResourceResidency;
	dest.shaderResourceMinLod = dest.shaderResourceMinLod || src.shaderResourceMinLod;
	dest.sparseBinding = dest.sparseBinding || src.sparseBinding;
	dest.sparseResidencyBuffer = dest.sparseResidencyBuffer || src.sparseResidencyBuffer;
	dest.sparseResidencyImage2D = dest.sparseResidencyImage2D || src.sparseResidencyImage2D;
	dest.sparseResidencyImage3D = dest.sparseResidencyImage3D || src.sparseResidencyImage3D;
	dest.sparseResidency2Samples = dest.sparseResidency2Samples || src.sparseResidency2Samples;
	dest.sparseResidency4Samples = dest.sparseResidency4Samples || src.sparseResidency4Samples;
	dest.sparseResidency8Samples = dest.sparseResidency8Samples || src.sparseResidency8Samples;
	dest.sparseResidency16Samples = dest.sparseResidency16Samples || src.sparseResidency16Samples;
	dest.sparseResidencyAliased = dest.sparseResidencyAliased || src.sparseResidencyAliased;
	dest.variableMultisampleRate = dest.variableMultisampleRate || src.variableMultisampleRate;
	dest.inheritedQueries = dest.inheritedQueries || src.inheritedQueries;
}

bool supports_features(const VkPhysicalDeviceFeatures& supported,
					   const VkPhysicalDeviceFeatures& requested,
					   const GenericFeatureChain& extension_supported,
					   const GenericFeatureChain& extension_requested) {

	if (requested.robustBufferAccess && !supported.robustBufferAccess) return false;
	if (requested.fullDrawIndexUint32 && !supported.fullDrawIndexUint32) return false;
	if (requested.imageCubeArray && !supported.imageCubeArray) return false;
	if (requested.independentBlend && !supported.independentBlend) return false;
	if (requested.geometryShader && !supported.geometryShader) return false;
	if (requested.tessellationShader && !supported.tessellationShader) return false;
	if (requested.sampleRateShading && !supported.sampleRateShading) return false;
	if (requested.dualSrcBlend && !supported.dualSrcBlend) return false;
	if (requested.logicOp && !supported.logicOp) return false;
	if (requested.multiDrawIndirect && !supported.multiDrawIndirect) return false;
	if (requested.drawIndirectFirstInstance && !supported.drawIndirectFirstInstance) return false;
	if (requested.depthClamp && !supported.depthClamp) return false;
	if (requested.depthBiasClamp && !supported.depthBiasClamp) return false;
	if (requested.fillModeNonSolid && !supported.fillModeNonSolid) return false;
	if (requested.depthBounds && !supported.depthBounds) return false;
	if (requested.wideLines && !supported.wideLines) return false;
	if (requested.largePoints && !supported.largePoints) return false;
	if (requested.alphaToOne && !supported.alphaToOne) return false;
	if (requested.multiViewport && !supported.multiViewport) return false;
	if (requested.samplerAnisotropy && !supported.samplerAnisotropy) return false;
	if (requested.textureCompressionETC2 && !supported.textureCompressionETC2) return false;
	if (requested.textureCompressionASTC_LDR && !supported.textureCompressionASTC_LDR) return false;
	if (requested.textureCompressionBC && !supported.textureCompressionBC) return false;
	if (requested.occlusionQueryPrecise && !supported.occlusionQueryPrecise) return false;
	if (requested.pipelineStatisticsQuery && !supported.pipelineStatisticsQuery) return false;
	if (requested.vertexPipelineStoresAndAtomics && !supported.vertexPipelineStoresAndAtomics) return false;
	if (requested.fragmentStoresAndAtomics && !supported.fragmentStoresAndAtomics) return false;
	if (requested.shaderTessellationAndGeometryPointSize && !supported.shaderTessellationAndGeometryPointSize) return false;
	if (requested.shaderImageGatherExtended && !supported.shaderImageGatherExtended) return false;
	if (requested.shaderStorageImageExtendedFormats && !supported.shaderStorageImageExtendedFormats) return false;
	if (requested.shaderStorageImageMultisample && !supported.shaderStorageImageMultisample) return false;
	if (requested.shaderStorageImageReadWithoutFormat && !supported.shaderStorageImageReadWithoutFormat) return false;
	if (requested.shaderStorageImageWriteWithoutFormat && !supported.shaderStorageImageWriteWithoutFormat) return false;
	if (requested.shaderUniformBufferArrayDynamicIndexing && !supported.shaderUniformBufferArrayDynamicIndexing) return false;
	if (requested.shaderSampledImageArrayDynamicIndexing && !supported.shaderSampledImageArrayDynamicIndexing) return false;
	if (requested.shaderStorageBufferArrayDynamicIndexing && !supported.shaderStorageBufferArrayDynamicIndexing) return false;
	if (requested.shaderStorageImageArrayDynamicIndexing && !supported.shaderStorageImageArrayDynamicIndexing) return false;
	if (requested.shaderClipDistance && !supported.shaderClipDistance) return false;
	if (requested.shaderCullDistance && !supported.shaderCullDistance) return false;
	if (requested.shaderFloat64 && !supported.shaderFloat64) return false;
	if (requested.shaderInt64 && !supported.shaderInt64) return false;
	if (requested.shaderInt16 && !supported.shaderInt16) return false;
	if (requested.shaderResourceResidency && !supported.shaderResourceResidency) return false;
	if (requested.shaderResourceMinLod && !supported.shaderResourceMinLod) return false;
	if (requested.sparseBinding && !supported.sparseBinding) return false;
	if (requested.sparseResidencyBuffer && !supported.sparseResidencyBuffer) return false;
	if (requested.sparseResidencyImage2D && !supported.sparseResidencyImage2D) return false;
	if (requested.sparseResidencyImage3D && !supported.sparseResidencyImage3D) return false;
	if (requested.sparseResidency2Samples && !supported.sparseResidency2Samples) return false;
	if (requested.sparseResidency4Samples && !supported.sparseResidency4Samples) return false;
	if (requested.sparseResidency8Samples && !supported.sparseResidency8Samples) return false;
	if (requested.sparseResidency16Samples && !supported.sparseResidency16Samples) return false;
	if (requested.sparseResidencyAliased && !supported.sparseResidencyAliased) return false;
	if (requested.variableMultisampleRate && !supported.variableMultisampleRate) return false;
	if (requested.inheritedQueries && !supported.inheritedQueries) return false;

	return extension_supported.match_all(extension_requested);
}
// clang-format on
// Finds the first queue which supports the desired operations. Returns QUEUE_INDEX_MAX_VALUE if none is found
uint32_t get_first_queue_index(std::vector<VkQueueFamilyProperties> const& families, VkQueueFlags desired_flags) {
    for (uint32_t i = 0; i < static_cast<uint32_t>(families.size()); i++) {
        if ((families[i].queueFlags & desired_flags) == desired_flags) return i;
    }
    return QUEUE_INDEX_MAX_VALUE;
}
// Finds the queue which is separate from the graphics queue and has the desired flag and not the
// undesired flag, but will select it if no better options are available compute support. Returns
// QUEUE_INDEX_MAX_VALUE if none is found.
uint32_t get_separate_queue_index(
    std::vector<VkQueueFamilyProperties> const& families, VkQueueFlags desired_flags, VkQueueFlags undesired_flags) {
    uint32_t index = QUEUE_INDEX_MAX_VALUE;
    for (uint32_t i = 0; i < static_cast<uint32_t>(families.size()); i++) {
        if ((families[i].queueFlags & desired_flags) == desired_flags && ((families[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) == 0)) {
            if ((families[i].queueFlags & undesired_flags) == 0) {
                return i;
            } else {
                index = i;
            }
        }
    }
    return index;
}

// finds the first queue which supports only the desired flag (not graphics or transfer). Returns QUEUE_INDEX_MAX_VALUE if none is found.
uint32_t get_dedicated_queue_index(
    std::vector<VkQueueFamilyProperties> const& families, VkQueueFlags desired_flags, VkQueueFlags undesired_flags) {
    for (uint32_t i = 0; i < static_cast<uint32_t>(families.size()); i++) {
        if ((families[i].queueFlags & desired_flags) == desired_flags &&
            (families[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) == 0 && (families[i].queueFlags & undesired_flags) == 0)
            return i;
    }
    return QUEUE_INDEX_MAX_VALUE;
}

// finds the first queue which supports presenting. returns QUEUE_INDEX_MAX_VALUE if none is found
uint32_t get_present_queue_index(
    VkPhysicalDevice const phys_device, VkSurfaceKHR const surface, std::vector<VkQueueFamilyProperties> const& families) {
    for (uint32_t i = 0; i < static_cast<uint32_t>(families.size()); i++) {
        VkBool32 presentSupport = false;
        if (surface != VK_NULL_HANDLE) {
            VkResult res = detail::vulkan_functions().fp_vkGetPhysicalDeviceSurfaceSupportKHR(phys_device, i, surface, &presentSupport);
            if (res != VK_SUCCESS) return QUEUE_INDEX_MAX_VALUE; // TODO: determine if this should fail another way
        }
        if (presentSupport == VK_TRUE) return i;
    }
    return QUEUE_INDEX_MAX_VALUE;
}
} // namespace detail

PhysicalDevice PhysicalDeviceSelector::populate_device_details(
    VkPhysicalDevice vk_phys_device, detail::GenericFeatureChain const& src_extended_features_chain) const {
    PhysicalDevice physical_device{};
    physical_device.physical_device = vk_phys_device;
    physical_device.surface = instance_info.surface;
    physical_device.defer_surface_initialization = criteria.defer_surface_initialization;
    physical_device.instance_version = instance_info.version;
    auto queue_families = detail::get_vector_noerror<VkQueueFamilyProperties>(
        detail::vulkan_functions().fp_vkGetPhysicalDeviceQueueFamilyProperties, vk_phys_device);
    physical_device.queue_families = queue_families;

    detail::vulkan_functions().fp_vkGetPhysicalDeviceProperties(vk_phys_device, &physical_device.properties);
    detail::vulkan_functions().fp_vkGetPhysicalDeviceFeatures(vk_phys_device, &physical_device.features);
    detail::vulkan_functions().fp_vkGetPhysicalDeviceMemoryProperties(vk_phys_device, &physical_device.memory_properties);

    physical_device.name = physical_device.properties.deviceName;

    std::vector<VkExtensionProperties> available_extensions;
    auto available_extensions_ret = detail::get_vector<VkExtensionProperties>(
        available_extensions, detail::vulkan_functions().fp_vkEnumerateDeviceExtensionProperties, vk_phys_device, nullptr);
    if (available_extensions_ret != VK_SUCCESS) return physical_device;
    for (const auto& ext : available_extensions) {
        physical_device.available_extensions.push_back(&ext.extensionName[0]);
    }

    physical_device.properties2_ext_enabled = instance_info.properties2_ext_enabled;

    auto fill_chain = src_extended_features_chain;

    bool instance_is_1_1 = instance_info.version >= VKB_VK_API_VERSION_1_1;
    if (!fill_chain.nodes.empty() && (instance_is_1_1 || instance_info.properties2_ext_enabled)) {
        VkPhysicalDeviceFeatures2 local_features{};
        fill_chain.chain_up(local_features);
        // Use KHR function if not able to use the core function
        if (instance_is_1_1) {
            detail::vulkan_functions().fp_vkGetPhysicalDeviceFeatures2(vk_phys_device, &local_features);
        } else {
            detail::vulkan_functions().fp_vkGetPhysicalDeviceFeatures2KHR(vk_phys_device, &local_features);
        }
        physical_device.extended_features_chain = fill_chain;
    }

    return physical_device;
}

PhysicalDevice::Suitable PhysicalDeviceSelector::is_device_suitable(PhysicalDevice const& pd) const {
    PhysicalDevice::Suitable suitable = PhysicalDevice::Suitable::yes;

    if (criteria.name.size() > 0 && criteria.name != pd.properties.deviceName) return PhysicalDevice::Suitable::no;

    if (criteria.required_version > pd.properties.apiVersion) return PhysicalDevice::Suitable::no;
    if (criteria.desired_version > pd.properties.apiVersion) suitable = PhysicalDevice::Suitable::partial;

    bool dedicated_compute = detail::get_dedicated_queue_index(pd.queue_families, VK_QUEUE_COMPUTE_BIT, VK_QUEUE_TRANSFER_BIT) !=
                             detail::QUEUE_INDEX_MAX_VALUE;
    bool dedicated_transfer = detail::get_dedicated_queue_index(pd.queue_families, VK_QUEUE_TRANSFER_BIT, VK_QUEUE_COMPUTE_BIT) !=
                              detail::QUEUE_INDEX_MAX_VALUE;
    bool separate_compute = detail::get_separate_queue_index(pd.queue_families, VK_QUEUE_COMPUTE_BIT, VK_QUEUE_TRANSFER_BIT) !=
                            detail::QUEUE_INDEX_MAX_VALUE;
    bool separate_transfer = detail::get_separate_queue_index(pd.queue_families, VK_QUEUE_TRANSFER_BIT, VK_QUEUE_COMPUTE_BIT) !=
                             detail::QUEUE_INDEX_MAX_VALUE;

    bool present_queue = detail::get_present_queue_index(pd.physical_device, instance_info.surface, pd.queue_families) !=
                         detail::QUEUE_INDEX_MAX_VALUE;

    if (criteria.require_dedicated_compute_queue && !dedicated_compute) return PhysicalDevice::Suitable::no;
    if (criteria.require_dedicated_transfer_queue && !dedicated_transfer) return PhysicalDevice::Suitable::no;
    if (criteria.require_separate_compute_queue && !separate_compute) return PhysicalDevice::Suitable::no;
    if (criteria.require_separate_transfer_queue && !separate_transfer) return PhysicalDevice::Suitable::no;
    if (criteria.require_present && !present_queue && !criteria.defer_surface_initialization)
        return PhysicalDevice::Suitable::no;

    auto required_extensions_supported =
        detail::check_device_extension_support(pd.available_extensions, criteria.required_extensions);
    if (required_extensions_supported.size() != criteria.required_extensions.size())
        return PhysicalDevice::Suitable::no;

    auto desired_extensions_supported = detail::check_device_extension_support(pd.available_extensions, criteria.desired_extensions);
    if (desired_extensions_supported.size() != criteria.desired_extensions.size())
        suitable = PhysicalDevice::Suitable::partial;

    if (!criteria.defer_surface_initialization && criteria.require_present) {
        std::vector<VkSurfaceFormatKHR> formats;
        std::vector<VkPresentModeKHR> present_modes;

        auto formats_ret = detail::get_vector<VkSurfaceFormatKHR>(formats,
            detail::vulkan_functions().fp_vkGetPhysicalDeviceSurfaceFormatsKHR,
            pd.physical_device,
            instance_info.surface);
        auto present_modes_ret = detail::get_vector<VkPresentModeKHR>(present_modes,
            detail::vulkan_functions().fp_vkGetPhysicalDeviceSurfacePresentModesKHR,
            pd.physical_device,
            instance_info.surface);

        if (formats_ret != VK_SUCCESS || present_modes_ret != VK_SUCCESS || formats.empty() || present_modes.empty()) {
            return PhysicalDevice::Suitable::no;
        }
    }

    if (!criteria.allow_any_type && pd.properties.deviceType != static_cast<VkPhysicalDeviceType>(criteria.preferred_type)) {
        suitable = PhysicalDevice::Suitable::partial;
    }

    bool required_features_supported = detail::supports_features(
        pd.features, criteria.required_features, pd.extended_features_chain, criteria.extended_features_chain);
    if (!required_features_supported) return PhysicalDevice::Suitable::no;

    for (uint32_t i = 0; i < pd.memory_properties.memoryHeapCount; i++) {
        if (pd.memory_properties.memoryHeaps[i].flags & VK_MEMORY_HEAP_DEVICE_LOCAL_BIT) {
            if (pd.memory_properties.memoryHeaps[i].size < criteria.required_mem_size) {
                return PhysicalDevice::Suitable::no;
            } else if (pd.memory_properties.memoryHeaps[i].size < criteria.desired_mem_size) {
                suitable = PhysicalDevice::Suitable::partial;
            }
        }
    }

    return suitable;
}
// delegate construction to the one with an explicit surface parameter
PhysicalDeviceSelector::PhysicalDeviceSelector(Instance const& instance)
: PhysicalDeviceSelector(instance, VK_NULL_HANDLE) {}

PhysicalDeviceSelector::PhysicalDeviceSelector(Instance const& instance, VkSurfaceKHR surface) {
    instance_info.instance = instance.instance;
    instance_info.version = instance.instance_version;
    instance_info.properties2_ext_enabled = instance.properties2_ext_enabled;
    instance_info.surface = surface;
    criteria.require_present = !instance.headless;
    criteria.required_version = instance.api_version;
    criteria.desired_version = instance.api_version;
}

Result<std::vector<PhysicalDevice>> PhysicalDeviceSelector::select_impl(DeviceSelectionMode selection) const {
#if !defined(NDEBUG)
    // Validation
    for (const auto& node : criteria.extended_features_chain.nodes) {
        assert(node.sType != static_cast<VkStructureType>(0) &&
               "Features struct sType must be filled with the struct's "
               "corresponding VkStructureType enum");
        assert(node.sType != VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2 &&
               "Do not pass VkPhysicalDeviceFeatures2 as a required extension feature structure. An "
               "instance of this is managed internally for selection criteria and device creation.");
    }
#endif

    if (criteria.require_present && !criteria.defer_surface_initialization) {
        if (instance_info.surface == VK_NULL_HANDLE)
            return Result<std::vector<PhysicalDevice>>{ PhysicalDeviceError::no_surface_provided };
    }

    // Get the VkPhysicalDevice handles on the system
    std::vector<VkPhysicalDevice> vk_physical_devices;

    auto vk_physical_devices_ret = detail::get_vector<VkPhysicalDevice>(
        vk_physical_devices, detail::vulkan_functions().fp_vkEnumeratePhysicalDevices, instance_info.instance);
    if (vk_physical_devices_ret != VK_SUCCESS) {
        return Result<std::vector<PhysicalDevice>>{ PhysicalDeviceError::failed_enumerate_physical_devices, vk_physical_devices_ret };
    }
    if (vk_physical_devices.size() == 0) {
        return Result<std::vector<PhysicalDevice>>{ PhysicalDeviceError::no_physical_devices_found };
    }

    auto fill_out_phys_dev_with_criteria = [&](PhysicalDevice& phys_dev) {
        phys_dev.features = criteria.required_features;
        phys_dev.extended_features_chain = criteria.extended_features_chain;
        bool portability_ext_available = false;
        for (const auto& ext : phys_dev.available_extensions)
            if (criteria.enable_portability_subset && ext == "VK_KHR_portability_subset")
                portability_ext_available = true;

        auto desired_extensions_supported =
            detail::check_device_extension_support(phys_dev.available_extensions, criteria.desired_extensions);

        phys_dev.extensions_to_enable.clear();
        phys_dev.extensions_to_enable.insert(
            phys_dev.extensions_to_enable.end(), criteria.required_extensions.begin(), criteria.required_extensions.end());
        phys_dev.extensions_to_enable.insert(
            phys_dev.extensions_to_enable.end(), desired_extensions_supported.begin(), desired_extensions_supported.end());
        if (portability_ext_available) {
            phys_dev.extensions_to_enable.push_back("VK_KHR_portability_subset");
        }
    };

    // if this option is set, always return only the first physical device found
    if (criteria.use_first_gpu_unconditionally && vk_physical_devices.size() > 0) {
        PhysicalDevice physical_device = populate_device_details(vk_physical_devices[0], criteria.extended_features_chain);
        fill_out_phys_dev_with_criteria(physical_device);
        return std::vector<PhysicalDevice>{ physical_device };
    }

    // Populate their details and check their suitability
    std::vector<PhysicalDevice> physical_devices;
    for (auto& vk_physical_device : vk_physical_devices) {
        PhysicalDevice phys_dev = populate_device_details(vk_physical_device, criteria.extended_features_chain);
        phys_dev.suitable = is_device_suitable(phys_dev);
        if (phys_dev.suitable != PhysicalDevice::Suitable::no) {
            physical_devices.push_back(phys_dev);
        }
    }

    // sort the list into fully and partially suitable devices. use stable_partition to maintain relative order
    const auto partition_index = std::stable_partition(physical_devices.begin(), physical_devices.end(), [](auto const& pd) {
        return pd.suitable == PhysicalDevice::Suitable::yes;
    });

    // Remove the partially suitable elements if they aren't desired
    if (selection == DeviceSelectionMode::only_fully_suitable) {
        physical_devices.erase(partition_index, physical_devices.end());
    }

    // Make the physical device ready to be used to create a Device from it
    for (auto& physical_device : physical_devices) {
        fill_out_phys_dev_with_criteria(physical_device);
    }

    return physical_devices;
}

Result<PhysicalDevice> PhysicalDeviceSelector::select(DeviceSelectionMode selection) const {
    auto const selected_devices = select_impl(selection);

    if (!selected_devices) return Result<PhysicalDevice>{ selected_devices.error() };
    if (selected_devices.value().size() == 0) {
        return Result<PhysicalDevice>{ PhysicalDeviceError::no_suitable_device };
    }

    return selected_devices.value().at(0);
}

// Return all devices which are considered suitable - intended for applications which want to let the user pick the physical device
Result<std::vector<PhysicalDevice>> PhysicalDeviceSelector::select_devices(DeviceSelectionMode selection) const {
    auto const selected_devices = select_impl(selection);
    if (!selected_devices) return Result<std::vector<PhysicalDevice>>{ selected_devices.error() };
    if (selected_devices.value().size() == 0) {
        return Result<std::vector<PhysicalDevice>>{ PhysicalDeviceError::no_suitable_device };
    }
    return selected_devices.value();
}

Result<std::vector<std::string>> PhysicalDeviceSelector::select_device_names(DeviceSelectionMode selection) const {
    auto const selected_devices = select_impl(selection);
    if (!selected_devices) return Result<std::vector<std::string>>{ selected_devices.error() };
    if (selected_devices.value().size() == 0) {
        return Result<std::vector<std::string>>{ PhysicalDeviceError::no_suitable_device };
    }
    std::vector<std::string> names;
    for (const auto& pd : selected_devices.value()) {
        names.push_back(pd.name);
    }
    return names;
}
PhysicalDeviceSelector& PhysicalDeviceSelector::set_surface(VkSurfaceKHR surface) {
    instance_info.surface = surface;
    return *this;
}
PhysicalDeviceSelector& PhysicalDeviceSelector::set_name(std::string const& name) {
    criteria.name = name;
    return *this;
}
PhysicalDeviceSelector& PhysicalDeviceSelector::prefer_gpu_device_type(PreferredDeviceType type) {
    criteria.preferred_type = type;
    return *this;
}
PhysicalDeviceSelector& PhysicalDeviceSelector::allow_any_gpu_device_type(bool allow_any_type) {
    criteria.allow_any_type = allow_any_type;
    return *this;
}
PhysicalDeviceSelector& PhysicalDeviceSelector::require_present(bool require) {
    criteria.require_present = require;
    return *this;
}
PhysicalDeviceSelector& PhysicalDeviceSelector::require_dedicated_transfer_queue() {
    criteria.require_dedicated_transfer_queue = true;
    return *this;
}
PhysicalDeviceSelector& PhysicalDeviceSelector::require_dedicated_compute_queue() {
    criteria.require_dedicated_compute_queue = true;
    return *this;
}
PhysicalDeviceSelector& PhysicalDeviceSelector::require_separate_transfer_queue() {
    criteria.require_separate_transfer_queue = true;
    return *this;
}
PhysicalDeviceSelector& PhysicalDeviceSelector::require_separate_compute_queue() {
    criteria.require_separate_compute_queue = true;
    return *this;
}
PhysicalDeviceSelector& PhysicalDeviceSelector::required_device_memory_size(VkDeviceSize size) {
    criteria.required_mem_size = size;
    return *this;
}
PhysicalDeviceSelector& PhysicalDeviceSelector::desired_device_memory_size(VkDeviceSize size) {
    criteria.desired_mem_size = size;
    return *this;
}
PhysicalDeviceSelector& PhysicalDeviceSelector::add_required_extension(const char* extension) {
    criteria.required_extensions.push_back(extension);
    return *this;
}
PhysicalDeviceSelector& PhysicalDeviceSelector::add_required_extensions(std::vector<const char*> const& extensions) {
    for (const auto& ext : extensions) {
        criteria.required_extensions.push_back(ext);
    }
    return *this;
}
PhysicalDeviceSelector& PhysicalDeviceSelector::add_required_extensions(size_t count, const char* const* extensions) {
    if (!extensions || count == 0) return *this;
    for (size_t i = 0; i < count; i++) {
        criteria.required_extensions.push_back(extensions[i]);
    }
    return *this;
}
PhysicalDeviceSelector& PhysicalDeviceSelector::add_desired_extension(const char* extension) {
    criteria.desired_extensions.push_back(extension);
    return *this;
}
PhysicalDeviceSelector& PhysicalDeviceSelector::add_desired_extensions(const std::vector<const char*>& extensions) {
    for (const auto& ext : extensions) {
        criteria.desired_extensions.push_back(ext);
    }
    return *this;
}
PhysicalDeviceSelector& PhysicalDeviceSelector::set_minimum_version(uint32_t major, uint32_t minor) {
    criteria.required_version = VKB_MAKE_VK_VERSION(0, major, minor, 0);
    return *this;
}
PhysicalDeviceSelector& PhysicalDeviceSelector::set_desired_version(uint32_t major, uint32_t minor) {
    criteria.desired_version = VKB_MAKE_VK_VERSION(0, major, minor, 0);
    return *this;
}
PhysicalDeviceSelector& PhysicalDeviceSelector::disable_portability_subset() {
    criteria.enable_portability_subset = false;
    return *this;
}

PhysicalDeviceSelector& PhysicalDeviceSelector::set_required_features(VkPhysicalDeviceFeatures const& features) {
    detail::combine_features(criteria.required_features, features);
    return *this;
}
#if defined(VKB_VK_API_VERSION_1_2)
// Just calls add_required_features
PhysicalDeviceSelector& PhysicalDeviceSelector::set_required_features_11(VkPhysicalDeviceVulkan11Features& features_11) {
    features_11.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_1_FEATURES;
    add_required_extension_features(features_11);
    return *this;
}
PhysicalDeviceSelector& PhysicalDeviceSelector::set_required_features_12(VkPhysicalDeviceVulkan12Features& features_12) {
    features_12.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES;
    add_required_extension_features(features_12);
    return *this;
}
#endif
#if defined(VKB_VK_API_VERSION_1_3)
PhysicalDeviceSelector& PhysicalDeviceSelector::set_required_features_13(VkPhysicalDeviceVulkan13Features& features_13) {
    features_13.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES;
    add_required_extension_features(features_13);
    return *this;
}
#endif
PhysicalDeviceSelector& PhysicalDeviceSelector::defer_surface_initialization() {
    criteria.defer_surface_initialization = true;
    return *this;
}
PhysicalDeviceSelector& PhysicalDeviceSelector::select_first_device_unconditionally(bool unconditionally) {
    criteria.use_first_gpu_unconditionally = unconditionally;
    return *this;
}

// PhysicalDevice
bool PhysicalDevice::has_dedicated_compute_queue() const {
    return detail::get_dedicated_queue_index(queue_families, VK_QUEUE_COMPUTE_BIT, VK_QUEUE_TRANSFER_BIT) != detail::QUEUE_INDEX_MAX_VALUE;
}
bool PhysicalDevice::has_separate_compute_queue() const {
    return detail::get_separate_queue_index(queue_families, VK_QUEUE_COMPUTE_BIT, VK_QUEUE_TRANSFER_BIT) != detail::QUEUE_INDEX_MAX_VALUE;
}
bool PhysicalDevice::has_dedicated_transfer_queue() const {
    return detail::get_dedicated_queue_index(queue_families, VK_QUEUE_TRANSFER_BIT, VK_QUEUE_COMPUTE_BIT) != detail::QUEUE_INDEX_MAX_VALUE;
}
bool PhysicalDevice::has_separate_transfer_queue() const {
    return detail::get_separate_queue_index(queue_families, VK_QUEUE_TRANSFER_BIT, VK_QUEUE_COMPUTE_BIT) != detail::QUEUE_INDEX_MAX_VALUE;
}
std::vector<VkQueueFamilyProperties> PhysicalDevice::get_queue_families() const { return queue_families; }
std::vector<std::string> PhysicalDevice::get_extensions() const { return extensions_to_enable; }
std::vector<std::string> PhysicalDevice::get_available_extensions() const { return available_extensions; }
bool PhysicalDevice::is_extension_present(const char* ext) const {
    return std::find_if(std::begin(available_extensions), std::end(available_extensions), [ext](std::string const& ext_name) {
        return ext_name == ext;
    }) != std::end(available_extensions);
}
bool PhysicalDevice::enable_extension_if_present(const char* extension) {
    auto it = std::find_if(std::begin(available_extensions),
        std::end(available_extensions),
        [extension](std::string const& ext_name) { return ext_name == extension; });
    if (it != std::end(available_extensions)) {
        extensions_to_enable.push_back(extension);
        return true;
    }
    return false;
}
bool PhysicalDevice::enable_extensions_if_present(const std::vector<const char*>& extensions) {
    for (const auto extension : extensions) {
        auto it = std::find_if(std::begin(available_extensions),
            std::end(available_extensions),
            [extension](std::string const& ext_name) { return ext_name == extension; });
        if (it == std::end(available_extensions)) return false;
    }
    for (const auto extension : extensions)
        extensions_to_enable.push_back(extension);
    return true;
}

bool PhysicalDevice::enable_features_if_present(const VkPhysicalDeviceFeatures& features_to_enable) {
    VkPhysicalDeviceFeatures actual_pdf{};
    detail::vulkan_functions().fp_vkGetPhysicalDeviceFeatures(physical_device, &actual_pdf);

    bool required_features_supported = detail::supports_features(actual_pdf, features_to_enable, {}, {});
    if (required_features_supported) {
        detail::combine_features(features, features_to_enable);
    }
    return required_features_supported;
}

bool PhysicalDevice::is_features_node_present(detail::GenericFeaturesPNextNode const& node) const {
    detail::GenericFeatureChain requested_features;
    requested_features.nodes.push_back(node);

    return extended_features_chain.find_and_match(requested_features);
}

bool PhysicalDevice::enable_features_node_if_present(detail::GenericFeaturesPNextNode const& node) {
    VkPhysicalDeviceFeatures2 actual_pdf2{};

    detail::GenericFeatureChain requested_features;
    requested_features.nodes.push_back(node);

    detail::GenericFeatureChain fill_chain = requested_features;
    // Zero out supported features
    memset(fill_chain.nodes.front().fields, UINT8_MAX, sizeof(VkBool32) * detail::GenericFeaturesPNextNode::field_capacity);

    actual_pdf2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
    fill_chain.chain_up(actual_pdf2);

    detail::vulkan_functions().fp_vkGetPhysicalDeviceFeatures2(physical_device, &actual_pdf2);
    bool required_features_supported = fill_chain.match_all(requested_features);
    if (required_features_supported) {
        extended_features_chain.combine(requested_features);
    }
    return required_features_supported;
}


PhysicalDevice::operator 
