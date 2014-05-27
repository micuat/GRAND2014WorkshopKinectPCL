# Attempt to load a config.make file.
# If none is found, project defaults in config.project.make will be used.
ifneq ($(wildcard config.make),)
	include config.make
endif

# make sure the the OF_ROOT location is defined
ifndef OF_ROOT
    OF_ROOT=../../..
endif

PROJECT_ADDONS_INCLUDES += /usr/include/pcl-1.7/
PROJECT_ADDONS_INCLUDES += /usr/include/eigen3/
PROJECT_ADDONS_LDFLAGS += -lboost_date_time
PROJECT_ADDONS_LDFLAGS += -lboost_filesystem
PROJECT_ADDONS_LDFLAGS += -lboost_iostreams
PROJECT_ADDONS_LDFLAGS += -lboost_system
PROJECT_ADDONS_LDFLAGS += -lboost_thread
PROJECT_ADDONS_LDFLAGS += -lpcl_common
PROJECT_ADDONS_LDFLAGS += -lpcl_features
PROJECT_ADDONS_LDFLAGS += -lpcl_filters
PROJECT_ADDONS_LDFLAGS += -lpcl_io
PROJECT_ADDONS_LDFLAGS += -lpcl_io_ply
PROJECT_ADDONS_LDFLAGS += -lpcl_kdtree
PROJECT_ADDONS_LDFLAGS += -lpcl_keypoints
PROJECT_ADDONS_LDFLAGS += -lpcl_octree
PROJECT_ADDONS_LDFLAGS += -lpcl_recognition
PROJECT_ADDONS_LDFLAGS += -lpcl_registration
PROJECT_ADDONS_LDFLAGS += -lpcl_sample_consensus
PROJECT_ADDONS_LDFLAGS += -lpcl_search
PROJECT_ADDONS_LDFLAGS += -lpcl_segmentation
PROJECT_ADDONS_LDFLAGS += -lpcl_surface
PROJECT_ADDONS_LDFLAGS += -lpcl_tracking

# call the project makefile!
include $(OF_ROOT)/libs/openFrameworksCompiled/project/makefileCommon/compile.project.mk
