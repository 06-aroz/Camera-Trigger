#define _CRT_SECURE_NO_WARNINGS 1

#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

#include <VmbCPP/VmbCPP.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <conio.h> // _kbhit/_getch for the stop key (Windows only)

// --------------------------------------------------------------------------
// TODO: X - Need to display camera properties (exposure time, pixel size, etc)
//		 X - Image formatting (time-name-[i].format)
//		 X - Generalize to 2 cameras
//		 Allow program to work outside of Vimba X examples folder
//		 !!External triggering!!
//
// ---------------------------------------------------------------------------

// Timeout is in MILLISECONDS and is per frame. Keep it separate from exposure time (which is in microseconds).
// With external triggering, this must be longer than the longest wait between triggers.
const VmbUint32_t DEFAULT_TIMEOUT = 2000;
const size_t DEFAULT_NUM_IM = 3;

struct prop_list_DOUBLES {
	std::string prop_name;
	double prop_val;
	bool can_change;
};

struct prop_list_STRINGS {
	std::string prop_name;
	std::string prop_val;
	bool can_change;
};


// We're defining the camera class for convienence
class Camera {
public:
	std::string name;
	std::string ID;
	std::string assign_name;
	std::string filepath;

	bool is_found = false;
	bool is_open = false;
	bool use_trigger = false; // true: captured in RunTriggerLoop on every trigger, false: captured once

	VmbCPP::CameraPtr camPtr;

	// PROP_TYPE, VALUE, IS_CHANGED
	std::vector<prop_list_DOUBLES> double_props;
	std::vector<prop_list_STRINGS> string_props;

	Camera() {
		camPtr = nullptr;
		is_found = false;
	}

	Camera(std::string cname, std::string cID, std::string ca_name) {
		name = cname;
		ID = cID;
		assign_name = ca_name;
	}

	void checkID() {
		std::string cam_ID;
		camPtr->GetID(cam_ID);
		if (ID == cam_ID) {
			std::cout << "Camera IDs match for " << name << std::endl;
			std::cout << " " << std::endl;
		}
		else {
			std::cout << "Camera IDs do not match for " << name << std::endl;
			std::cout << "Pointer ID: " << cam_ID << std::endl;
			std::cout << "Camera ID: " << ID << std::endl;
			std::cout << " " << std::endl;
		}
	}

	void isFound() { is_found = camPtr != nullptr; }
};

class CamSystem {
public:
	bool is_off = true;
	bool change_vals = false;
	VmbUint32_t camera_timeout = DEFAULT_TIMEOUT;
	size_t num_images = DEFAULT_NUM_IM;
	std::vector<Camera> cameras = {};

	CamSystem() { is_off = false; }

};

// Returns false on failure instead of throwing, so the caller can still close the camera
bool GigEAdjustPacketSize(VmbCPP::CameraPtr camera) {
	VmbCPP::StreamPtrVector streams;
	VmbErrorType err = camera->GetStreams(streams);

	if (err != VmbErrorSuccess || streams.empty()) {
		std::cerr << "Could not get stream modules. Error code: " << err << std::endl;
		return false; // streams[0] below would crash on an empty list
	}

	VmbCPP::FeaturePtr feature;
	err = streams[0]->GetFeatureByName("GVSPAdjustPacketSize", feature);

	if (err == VmbErrorSuccess) {
		err = feature->RunCommand();
		if (err == VmbErrorSuccess) {
			bool commandDone = false;
			do {
				if (feature->IsCommandDone(commandDone) != VmbErrorSuccess) {
					break;
				}
			} while (commandDone == false);
		}
		else {
			std::cerr << "Error while executing GVSPAdjustPacketSize. Error code: " << err << std::endl;
			return false;
		}
	}
	return true;
}

cv::Mat ConvertVimbaToMat(const VmbCPP::FramePtr& frame) {
	if (!frame) {
		std::cerr << "Frame doesn't exist!" << std::endl;
		return {};
	}

	VmbUint32_t width = 0;
	VmbUint32_t height = 0;

	frame->GetWidth(width);
	frame->GetHeight(height);

	VmbFrameStatusType status;
	frame->GetReceiveStatus(status);

	if (status != VmbFrameStatusComplete) {
		std::cerr << "Frame incomplete! Error code: " << status << std::endl;
		return {};
	}

	// This conversion assumes Mono8 (1 byte per pixel). Other formats would come out garbled.
	VmbPixelFormatType pixel_format;
	frame->GetPixelFormat(pixel_format);
	if (pixel_format != VmbPixelFormatMono8) {
		std::cerr << "Warning: frame is not Mono8 (pixel format code: " << pixel_format << "). Image may be wrong." << std::endl;
	}

	VmbUchar_t* buffer = nullptr;
	VmbErrorType err = frame->GetImage(buffer);

	if (err != VmbErrorSuccess || buffer == nullptr) {
		std::cerr << "No image buffer\n";
		return {};
	}

	return cv::Mat(height, width, CV_8UC1, buffer).clone();
}

std::string getTime() {
	// Time formatting
	auto now_tp = std::chrono::system_clock::now();
	std::time_t now = std::chrono::system_clock::to_time_t(now_tp);
	std::tm* localTime = std::localtime(&now);
	int ms = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(now_tp.time_since_epoch()).count() % 1000);

	char buffer[80];
	// Format: Year-Month-Day_Hour.Minute.Second.Millisecond (ms keeps triggers within the same second apart)
	std::strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H.%M.%S", localTime);
	char ms_buffer[8];
	std::snprintf(ms_buffer, sizeof(ms_buffer), ".%03d", ms);
	return std::string(buffer) + ms_buffer;
}

void FindCams(VmbCPP::CameraPtrVector &cameras) {
	// Listing all cameras and their properties
	std::size_t cam_list_length = cameras.size();
	for (size_t i = 0; i < cam_list_length; i++) {
		VmbCPP::CameraPtr camera = cameras[i];

		std::string cam_ID;
		std::string cam_name;

		camera->GetID(cam_ID);
		camera->GetName(cam_name);

		std::cout << "Found camera ID: " << cam_ID << std::endl;
		std::cout << "Found camera name: " << cam_name << std::endl;
		std::cout << " " << std::endl;
	}
}

// Integer features (e.g. AcquisitionFrameCount) must be read/written as VmbInt64_t, float features as double.
// These helpers let the double prop list handle both kinds.
bool IsIntFeature(VmbCPP::FeaturePtr pFeat) {
	VmbFeatureDataType type = VmbFeatureDataUnknown;
	pFeat->GetDataType(type);
	return type == VmbFeatureDataInt;
}

void GetNumber(VmbCPP::FeaturePtr pFeat, double& val) {
	if (IsIntFeature(pFeat)) {
		VmbInt64_t ival = 0;
		pFeat->GetValue(ival);
		val = static_cast<double>(ival);
	}
	else {
		pFeat->GetValue(val);
	}
}

void GetNumberRange(VmbCPP::FeaturePtr pFeat, double& min, double& max) {
	if (IsIntFeature(pFeat)) {
		VmbInt64_t imin = 0, imax = 0;
		pFeat->GetRange(imin, imax);
		min = static_cast<double>(imin);
		max = static_cast<double>(imax);
	}
	else {
		pFeat->GetRange(min, max);
	}
}

VmbErrorType SetNumber(VmbCPP::FeaturePtr pFeat, double val) {
	if (IsIntFeature(pFeat)) {
		return pFeat->SetValue(static_cast<VmbInt64_t>(val));
	}
	return pFeat->SetValue(val);
}

bool PrintPropDouble(VmbCPP::CameraPtr cam, const std::string& feat_type, bool show_range) {
	if (cam != nullptr) {
		VmbCPP::FeaturePtr pFeat;
		VmbErrorType err = cam->GetFeatureByName(feat_type.c_str(), pFeat);
		if (VmbErrorSuccess == err) {
			double cur_val = 0.0;
			std::string units;
			// When you do this, you are using the pointer's method to set your variable to whatever the pointer finds
			GetNumber(pFeat, cur_val);
			pFeat->GetUnit(units);
			std::cout << "Current "<< feat_type <<" value: " << cur_val << " " << units << std::endl;
			if (show_range) {
				double min = 0.0, max = 0.0;
				GetNumberRange(pFeat, min, max);
				std::cout << ">>> Range for " << feat_type << " " << min << "-" << max << " " << units << std::endl;
			}
			return false; // Successful find, then find_err = false
		}
		else {
			std::cerr << "Failed to find " << feat_type << " feature. Error code: " << err << std::endl;
			return true;
		}
	}
	else {
		std::cout << "Unable to display feature. Camera pointer is null." << std::endl;
		return true;
	}
}

bool PrintPropString(VmbCPP::CameraPtr cam, const std::string &feat_type) {
	if (cam != nullptr) {
		VmbCPP::FeaturePtr pFeat;
		VmbErrorType err = cam->GetFeatureByName(feat_type.c_str(), pFeat);
		if (VmbErrorSuccess == err) {
			std::string curr_val;
			pFeat->GetValue(curr_val);
			std::cout << "Current " << feat_type << " value: " << curr_val << std::endl;
			return false; // Successful find, then find_err = false
		}
		else {
			std::cerr << "Failed to find " << feat_type << " feature. Error code: " << err << std::endl;
			return true;
		}
	}
	else {
		std::cout << "Unable to display feature. Camera pointer is null." << std::endl;
		return true;
	}
}

void SetPropDouble(VmbCPP::CameraPtr cam, double set, const std::string& feat_type) {
	VmbCPP::FeaturePtr pFeat;
	VmbErrorType err = cam->GetFeatureByName(feat_type.c_str(), pFeat);
	if (VmbErrorSuccess == err) {
		double min = 0.0, max = 0.0;
		GetNumberRange(pFeat, min, max);
		if (set >= min && set <= max) {
			VmbErrorType set_err = SetNumber(pFeat, set);
			if (set_err != VmbErrorSuccess) {
				std::cerr << "Error setting " << feat_type << " to: " << set << ". Error code: " << set_err << std::endl;
				return;
			}
			std::string units;
			pFeat->GetUnit(units);
			std::cout << "Set " << feat_type << " to: " << set << " " << units << std::endl;
		}
		else {
			std::cout << "Set value " << set << " is outside of feature range: (" << min << ", " << max << ")\n";
		}
	}
}

void SetPropString(VmbCPP::CameraPtr cam, std::string set, const std::string& feat_type) {
	VmbCPP::FeaturePtr pFeat;
	VmbErrorType err = cam->GetFeatureByName(feat_type.c_str(), pFeat);
	if (VmbErrorSuccess == err) {
		VmbErrorType set_err = pFeat->SetValue(set.c_str());
		if (set_err != VmbErrorSuccess) {
			std::cerr << "Error setting " << feat_type << " to: " << set << ". Error code: " << set_err << std::endl;
			return;
		}
		std::cout << "Set " << feat_type << " to: " << set << std::endl;
	}
}

void ListCamProps(CamSystem& cam_system) {
	for (const auto& camera : cam_system.cameras) {
		VmbCPP::CameraPtr cam = camera.camPtr;
		if (cam != nullptr && camera.is_open) {
			// If you get an error code of -3, it's probably because you typed in the wrong feature name
			std::cout << "\nDisplaying current feature values for [" << camera.assign_name << "]\n-------------------------------------" << std::endl;

			// String (enum) props go FIRST so modes like ExposureAuto=Off are applied before the numeric values
			for (size_t i = 0; i < camera.string_props.size(); i++) {
				const prop_list_STRINGS& p = camera.string_props[i];
				bool find_err = PrintPropString(cam, p.prop_name); // Checked per feature, so one bad name doesn't block the rest
				// Conditional to change values
				if (cam_system.change_vals && !find_err && p.can_change) {
					SetPropString(cam, p.prop_val, p.prop_name);
					PrintPropString(cam, p.prop_name); // Read back what the camera actually accepted
				}
			}

			for (size_t i = 0; i < camera.double_props.size(); i++) {
				const prop_list_DOUBLES& p = camera.double_props[i];
				bool find_err = PrintPropDouble(cam, p.prop_name, true);
				// Conditional to change values
				if (cam_system.change_vals && !find_err && p.can_change) {
					SetPropDouble(cam, p.prop_val, p.prop_name);
					PrintPropDouble(cam, p.prop_name, false); // Read back what the camera actually accepted
				}
			}
		}
	}
}

void CheckMainCameras(CamSystem& cam_system, VmbCPP::VmbSystem& sys) {
	// Checking for all cameras
	VmbCPP::CameraPtrVector hardware_cams;
	VmbErrorType err = sys.GetCameras(hardware_cams);

	if (err != VmbErrorSuccess || hardware_cams.empty()) {
		std::cerr << "No physical cameras available to check." << std::endl;
		return;
	}

	// Checking for the cameras we will actually use
	std::size_t cam_list_length = hardware_cams.size();
	for (size_t i = 0; i < cam_list_length; i++) {
		VmbCPP::CameraPtr camera = hardware_cams[i];

		std::string cam_ID;
		camera->GetID(cam_ID);

		for (auto& cam : cam_system.cameras) {
			if (cam_ID == cam.ID) {
				cam.camPtr = camera;
				cam.checkID();
				cam.isFound();
			}
		}
	}
}

void OpenCameras(CamSystem &cam_system, VmbCPP::VmbSystem& sys) {
	for (auto& cam : cam_system.cameras) {

		if (cam.is_found) {
			VmbErrorType err = sys.OpenCameraByID(cam.ID, VmbAccessModeFull, cam.camPtr);
			if (err == VmbErrorSuccess) {
				std::cout << "Camera [" << cam.assign_name << "] opened succesfully." << std::endl;
				std::cout << " " << std::endl;
				cam.is_open = true;
				GigEAdjustPacketSize(cam.camPtr);
			}
			else {
				std::cerr << "Error! Cannot open camera [" << cam.assign_name << "]. Error code: " << err << std::endl;
				std::cout << "" << std::endl;
			}
		}
		else {
			std::cout << "Did not open camera [" << cam.assign_name << "] cannot be found" << std::endl;
		}
	}
}

// Captures num_images frames from one camera and saves them.
// Returns false if no frame arrived before the timeout (for a triggered camera: no trigger yet).
bool CaptureBurst(CamSystem& cam_system, Camera& cam) {
	// imwrite fails if the folder doesn't exist
	if (!cv::utils::fs::createDirectories(cam.filepath)) {
		std::cerr << "Could not create image folder " << cam.filepath << std::endl;
		return false;
	}

	const size_t num_im_cap = cam_system.num_images;
	VmbCPP::FramePtrVector frame_batch(num_im_cap);
	VmbUint32_t completed_frcount = 0;

	VmbErrorType grab_err = cam.camPtr->AcquireMultipleImages(
		frame_batch,
		cam_system.camera_timeout,
		completed_frcount
	);

	if (grab_err == VmbErrorTimeout && completed_frcount == 0) {
		return false; // Nothing arrived, not an error while waiting for a trigger
	}

	if (grab_err == VmbErrorSuccess) {
		std::cout << "Successfully filled all " << completed_frcount << " buffers for " << cam.assign_name << std::endl;
	}
	else {
		std::cerr << "AcquireMultipleImages exited with code: " << grab_err << ". Completed frames: " << completed_frcount << "/" << num_im_cap << std::endl;
	}

	size_t saved_count = 0;
	const std::string time_stamp = getTime(); // Same timestamp for every image in this batch
	// Buffer loop
	for (size_t i = 0; i < completed_frcount; i++) {
		// -- Processing --
		cv::Mat con_frame = ConvertVimbaToMat(frame_batch[i]);
		if (con_frame.empty()) {
			std::cerr << "Skipping frame " << i << " for " << cam.assign_name << " (could not convert)" << std::endl;
			continue;
		}

		std::string name = time_stamp + "_" + cam.assign_name + "_image_" + std::to_string(i) + ".bmp";
		std::string file_path = cam.filepath + name;
		if (cv::imwrite(file_path, con_frame)) {
			saved_count++;
		}
		else {
			std::cerr << "ERROR! Failed to write " << file_path << std::endl;
		}
	}
	std::cout << "Saved " << saved_count << "/" << completed_frcount << " image(s) to " << cam.filepath << '\n' << std::endl;
	return true;
}

// One capture from each open camera that is NOT waiting for a trigger (free-running cameras)
// Note: cameras are captured one after another, not at the same time
void RunSyncCapture(CamSystem &cam_system) {
	for (auto& cam : cam_system.cameras) {
		if (!cam.is_open) {
			std::cerr << "Cam " << cam.assign_name << " is not open!" << std::endl;
			continue;
		}
		if (cam.use_trigger) continue; // Handled by RunTriggerLoop

		std::cout << "Attempting to synchronously capture " << cam_system.num_images << " frames for " << cam.assign_name << "...\n" << std::endl;
		if (!CaptureBurst(cam_system, cam)) {
			std::cerr << "No frames from " << cam.assign_name << " before the timeout" << std::endl;
		}
	}
}

// Keeps waiting for triggers on every open camera with use_trigger = true, until q or Esc is pressed.
// The key is checked after each wait, so stopping can take up to camera_timeout.
void RunTriggerLoop(CamSystem& cam_system) {
	bool any_triggered = false;
	for (const auto& cam : cam_system.cameras) {
		if (cam.is_open && cam.use_trigger) any_triggered = true;
	}
	if (!any_triggered) {
		std::cout << "No open cameras with use_trigger = true, skipping trigger loop" << std::endl;
		return;
	}

	std::cout << "\nWaiting for triggers... press q or Esc to stop.\n" << std::endl;
	size_t burst_count = 0;
	bool keep_running = true;
	while (keep_running) {
		for (auto& cam : cam_system.cameras) {
			if (cam.is_open && cam.use_trigger) {
				if (CaptureBurst(cam_system, cam)) burst_count++;
			}
		}

		// _kbhit doesn't block: it only reports whether a key is waiting
		while (_kbhit()) {
			int key = _getch();
			if (key == 'q' || key == 'Q' || key == 27) keep_running = false; // 27 = Esc
		}
	}
	std::cout << "Stopped trigger loop after " << burst_count << " burst(s)." << std::endl;
}


void CloseSystem(CamSystem &cam_system, VmbCPP::VmbSystem& sys) {
	for (auto& cam : cam_system.cameras) {
		if (cam.is_open) {
			cam.camPtr->Close();
			cam.is_open = false;
		}
	}

	sys.Shutdown();
	std::cout << "=========================" << std::endl;
	std::cout << "  Shut down Vimba X API" << std::endl;
	std::cout << "=========================" << std::endl;
}

int main() {
	VmbCPP::VmbSystem& sys = VmbCPP::VmbSystem::GetInstance();
	CamSystem LAB_SYS;

	// Get Vimba X version, important for debugging
	VmbVersionInfo_t versionInfo;
	VmbErrorType ver_err = sys.QueryVersion(versionInfo);

	if (ver_err == VmbErrorSuccess) {
		std::cout << "Vimba X C++ API Version: "
			<< versionInfo.major << "."
			<< versionInfo.minor << "."
			<< versionInfo.patch << std::endl;
	}
	else {
		std::cerr << "Failed to query version. Error code: " << ver_err << std::endl;
	}

	// To disable openCV informational status messages
	cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_WARNING);

	VmbErrorType err = sys.Startup();
	if (err != VmbErrorSuccess) {
		std::cerr << "Failed to launch Vimba X. Error code: " << err << std::endl;
		return 1;
	}
	std::cout << "Vimba X launched succesfully!" << std::endl;
	std::cout << "=============================" << std::endl;

	// Here we define our vector of camera pointers and use it for error handling
	VmbCPP::CameraPtrVector cameras;

	// Did we get an error finding a camera?
	err = sys.GetCameras(cameras);
	if (err != VmbErrorSuccess) {
		std::cout << "Failed to retrieve cameras. Error code:" << err << std::endl;
		sys.Shutdown();
		return 0;
	}

	// Checking if there simply are no cameras
	if (cameras.empty()) {
		std::cout << "No cameras found!" << std::endl;
		sys.Shutdown();
		return 0;
	}

	// Defining our cameras for the G1 and Manta
	// String props are applied first (in list order), then double props.
	Camera Manta;
	Manta.name = "Allied Vision Technologies Manta G-419B-NIR (E0022528) (DEV_000F31504D3C)";
	Manta.ID = "DEV_000F31504D3C";
	Manta.assign_name = "Manta";
	Manta.filepath = "./M_images/";
	Manta.use_trigger = true; // Captured on every Line1 pulse until q/Esc
	Manta.string_props = {
		// UserSetSelector only picks a set; loading it also needs the UserSetLoad command, so it is left out for now
		{"ExposureAuto", "Off", true},
		{"PixelFormat", "Mono8", true},
		// --- One pulse on Line1 -> a burst of AcquisitionFrameCount frames ---
		// FrameStart trigger off: once acquisition starts, frames come at AcquisitionFrameRateAbs
		{"TriggerSelector", "FrameStart", true},
		{"TriggerMode", "Off", true},
		// AcquisitionStart trigger on Line1: the camera waits for the pulse before starting
		{"TriggerSelector", "AcquisitionStart", true},
		{"TriggerSource", "Line1", true},
		{"TriggerActivation", "RisingEdge", true},
		{"TriggerMode", "On", true},
		// MultiFrame: stop by itself after AcquisitionFrameCount frames
		{"AcquisitionMode", "MultiFrame", true},
		// For one frame per pulse instead, use only: TriggerSelector=FrameStart, TriggerSource=Line1, TriggerMode=On, AcquisitionMode=Continuous
	};

	Manta.double_props = {
		{"ExposureTimeAbs", 5000, true },
		{"AcquisitionFrameRateAbs", 26.5, false },	// Spacing between burst frames. Set to true to control it
		{"AcquisitionFrameCount", static_cast<double>(DEFAULT_NUM_IM), true },	// Frames per trigger; matches num_images
		{"Gain", 0.0, true },
	};

	Camera Alvium_G1;
	Alvium_G1.name = "Allied Vision Alvium G1-812 UV (DEV_000A4710218A)";
	Alvium_G1.ID = "DEV_000A4710218A";
	Alvium_G1.assign_name = "Alvium_G1";
	Alvium_G1.filepath = "./G1_images/";
	Alvium_G1.string_props = {
		{"ExposureAuto", "Off", true},
		{"PixelFormat", "Mono8", true},
		{"TriggerMode", "Off", true},
		{"TriggerSource", "Software", true},
		// External trigger: replace the two trigger lines above with these (check line names in Vimba X Viewer)
		//{"TriggerSelector", "FrameStart", true},
		//{"TriggerSource", "Line0", true},
		//{"TriggerMode", "On", true},	// set last
	};

	Alvium_G1.double_props = {
		{"ExposureTime", 5000, true },
		{"AcquisitionFrameRate", 10, false },	// Changing this also needs AcquisitionFrameRateEnable = true
		{"Gain", 0.0, true },
	};

	LAB_SYS.cameras = {Manta, Alvium_G1};
	LAB_SYS.change_vals = true;
	// ms, per frame. Also how often the trigger loop checks for the stop key, so keep it short.
	// Must stay longer than the gap between burst frames (1 / AcquisitionFrameRateAbs).
	LAB_SYS.camera_timeout = 1000;

	FindCams(cameras);

	// Listing all cameras
	CheckMainCameras(LAB_SYS, sys);
	// Opening a camera
	OpenCameras(LAB_SYS, sys);
	ListCamProps(LAB_SYS);
	// One capture from free-running cameras
	RunSyncCapture(LAB_SYS);
	// Capture on every trigger until q/Esc
	RunTriggerLoop(LAB_SYS);
	// Closing system
	CloseSystem(LAB_SYS, sys);
	return 0;
}
