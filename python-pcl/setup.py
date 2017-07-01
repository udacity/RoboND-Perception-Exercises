# -*- coding: utf-8 -*-
from __future__ import print_function
from collections import defaultdict
from Cython.Distutils import build_ext
from distutils.core import setup
from distutils.extension import Extension
# from Cython.Build import cythonize    # MacOS NG
from setuptools import setup, find_packages, Extension

import subprocess
import numpy
import sys
import platform
import os
import time

sys.path.append('./pcl')
sys.path.append('./tests')

setup_requires = []
install_requires = [
    'filelock',
    'nose',
    'numpy',
    'Cython>=0.25.2',
]

if platform.system() == "Windows":
    # Check 32bit or 64bit
    is_64bits = sys.maxsize > 2**32
    # if is_64bits == True
    
    # environment Value
    for k, v in os.environ.items():
        # print("{key} : {value}".format(key=k, value=v))
        if k == "PCL_ROOT":
            pcl_root = v
            # print(pcl_root)
            # print("%s: find environment PCL_ROOT" % pcl_root)
            break
    else:
        print("cannot find environment PCL_ROOT", file=sys.stderr)
        sys.exit(1)
    
    # Add environment Value
    for k, v in os.environ.items():
        # print("{key} : {value}".format(key=k, value=v))
        if k == "PKG_CONFIG_PATH":
            pkgconfigstr = v
            break
    else:
        # print("cannot find environment PKG_CONFIG_PATH", file=sys.stderr)
        print("cannot find environment PKG_CONFIG_PATH")
        pkgconfigstr = pcl_root + '\\lib\\pkgconfig;' + pcl_root + '\\3rdParty\\FLANN\\lib\\pkgconfig;' + pcl_root + '\\3rdParty\\Eigen\\lib\\pkgconfig;'
        os.environ["PKG_CONFIG_PATH"] = pcl_root + '\\lib\\pkgconfig;' + pcl_root + '\\3rdParty\\FLANN\\lib\\pkgconfig;' + pcl_root + '\\3rdParty\\Eigen\\lib\\pkgconfig;'
    
    print("set environment PKG_CONFIG_PATH=%s" % pkgconfigstr)
    
    # other package(common)
    # BOOST_ROOT
    for k, v in os.environ.items():
        # print("{key} : {value}".format(key=k, value=v))
        if k == "BOOST_ROOT":
            boost_root = v
            break
    else:
        boost_root = pcl_root + '\\3rdParty\\Boost'
    
    # EIGEN_ROOT
    for k, v in os.environ.items():
        # print("{key} : {value}".format(key=k, value=v))
        if k == "EIGEN_ROOT":
            eigen_root = v
            break
    else:
        eigen_root = pcl_root + '\\3rdParty\\Eigen'
    
    # FLANN_ROOT
    for k, v in os.environ.items():
        # print("{key} : {value}".format(key=k, value=v))
        if k == "FLANN_ROOT":
            flann_root = v
            break
    else:
        flann_root = pcl_root + '\\3rdParty\\FLANN'
    
    # QHULL_ROOT
    for k, v in os.environ.items():
        # print("{key} : {value}".format(key=k, value=v))
        if k == "QHULL_ROOT":
            qhull_root = v
            break
    else:
        qhull_root = pcl_root + '\\3rdParty\\Eigen'

    # VTK_DIR
    for k, v in os.environ.items():
        # print("{key} : {value}".format(key=k, value=v))
        if k == "VTK_DIR":
            vtk_root = v
            break
    else:
        vtk_root = pcl_root + '\\3rdParty\\VTK'
    
    # custom(CUDA)
    # custom(WinPcap)
    
    # get pkg-config.exe filePath
    pkgconfigPath = os.getcwd() + '\\pkg-config\\pkg-config.exe'
    print(pkgconfigPath)
    
    # AppVeyor Check
    for k, v in os.environ.items():
        # print("{key} : {value}".format(key=k, value=v))
        if k == "PCL_VERSION":
            pcl_version = '-' + v
            break
    else:
        # Try to find PCL. XXX we should only do this when trying to build or install.
        PCL_SUPPORTED = ["-1.9", "-1.8", "-1.7", ""]    # in order of preference
        
        for pcl_version in PCL_SUPPORTED:
            if subprocess.call(['.\\pkg-config\\pkg-config.exe', 'pcl_common%s' % pcl_version]) == 0:
            # if subprocess.call([pkgconfigPath, 'pcl_common%s' % pcl_version]) == 0:
                break
        else:
            print("%s: error: cannot find PCL, tried" % sys.argv[0], file=sys.stderr)
            for version in PCL_SUPPORTED:
                print('    pkg-config pcl_common%s' % version, file=sys.stderr)
            sys.exit(1)
    
    print(pcl_version)
    # pcl_version = '-1.7'
    
    # Python Version Check
    info = sys.version_info
    
    if pcl_version == '-1.7':
        # PCL 1.7.2 python Version >= 3.5
        # Visual Studio 2015
        if info.major == 3 and info.minor >= 5:
            boost_version = '1_57'
            vtk_version = '6.2'
            pass
        else:
            print('no building Python Version')
            sys.exit(1)
    elif pcl_version == '-1.8':
        # PCL 1.8.0 python Version >= 3.5
        # Visual Studio 2015
        if info.major == 3 and info.minor >= 5:
            boost_version = '1_61'
            vtk_version = '7.0'
        else:
            print('no building Python Version')
            sys.exit(1)
    elif pcl_version == '-1.9':
        # PCL 1.9.0 python Version >= 3.5
        # Visual Studio 2015
        if info.major == 3 and info.minor >= 5:
            boost_version = '1_61'
            vtk_version = '7.0'
            pass
        else:
            print('no building Python Version')
            sys.exit(1)
    else:
        print('pcl_version Unknown')
        sys.exit(1)
    
    # Find build/link options for PCL using pkg-config.
    pcl_libs = ["common", "features", "filters", "kdtree", "octree",
                "registration", "sample_consensus", "search", "segmentation",
                "surface", "tracking", "visualization"]
    
    # pcl-1.7
    # pcl_libs = ["common", "features", "filters", "geometry", 
    #             "io", "kdtree", "keypoints", "octree", "outofcore", "people", 
    #             "recognition", "registration", "sample_consensus", "search", 
    #             "segmentation", "surface", "tracking", "visualization"]
    
    # pcl-1.8
    # pcl_libs = ["2d", "common", "features", "filters", "geometry", 
    #             "io", "kdtree", "keypoints", "ml", "octree", "outofcore", "people", 
    #             "recognition", "registration", "sample_consensus", "search", 
    #             "segmentation", "stereo", "surface", "tracking", "visualization"]
    
    pcl_libs = ["pcl_%s%s" % (lib, pcl_version) for lib in pcl_libs]
    # pcl_libs += ['Eigen3']
    # print(pcl_libs)
    
    ext_args = defaultdict(list)
    # set include path
    ext_args['include_dirs'].append(numpy.get_include())
    
    def pkgconfig(flag, cut):
        # Equivalent in Python 2.7 (but not 2.6):
        # subprocess.check_output(['pkg-config', flag] + pcl_libs).split()
        p = subprocess.Popen(['pkg-config', flag] + pcl_libs, stdout=subprocess.PIPE)
        stdout, _ = p.communicate()
        # Assume no evil spaces in filenames; unsure how pkg-config would
        # handle those, anyway.
        # decode() is required in Python 3. TODO how do know the encoding?
        # return stdout.decode().split()
        # Windows
        # return stdout.decode().replace('\r\n', '').replace('\ ', ' ').replace('/', '\\').split(cut)
        return stdout.decode().replace('\r\n', '').replace('\ ', ' ').replace('/', '\\').split(cut)
    
    # Get setting pkg-config
    # use pkg-config
    # # start
    # for flag in pkgconfig('--cflags-only-I', '-I'):
    #     print(flag.lstrip().rstrip())
    #     ext_args['include_dirs'].append(flag.lstrip().rstrip())
    # 
    # end
    # no use pkg-config
    inc_dirs = [pcl_root + '\\include\\pcl' + pcl_version, eigen_root + '\\eigen3', boost_root + '\\include\\boost-' + boost_version, flann_root + '\\include', vtk_root + '\\include\\vtk-' + vtk_version]
    
    for inc_dir in inc_dirs:
        ext_args['include_dirs'].append(inc_dir)
    
    # for flag in pkgconfig('--libs-only-L'):
    #     ext_args['library_dirs'].append(flag[2:])
    # 
    # for flag in pkgconfig('--libs-only-other'):
    #     ext_args['extra_link_args'].append(flag)
    # end
    
    # set library path
    lib_dirs = [pcl_root + '\\lib', boost_root + '\\lib', flann_root + '\\lib', vtk_root + '\\lib']
    
    for lib_dir in lib_dirs:
        ext_args['library_dirs'].append(lib_dir)
    
    # OpenNI2?
    # %OPENNI2_REDIST64% %OPENNI2_REDIST%
    
    # set compiler flags
    # for flag in pkgconfig('--cflags-only-other'):
    #     if flag.startswith('-D'):
    #         macro, value = flag[2:].split('=', 1)
    #         ext_args['define_macros'].append((macro, value))
    #     else:
    #         ext_args['extra_compile_args'].append(flag)
    # 
    # for flag in pkgconfig('--libs-only-l', '-l'):
    #     if flag == "-lflann_cpp-gd":
    #         print("skipping -lflann_cpp-gd (see https://github.com/strawlab/python-pcl/issues/29")
    #         continue
    #     ext_args['libraries'].append(flag.lstrip().rstrip())
    
    if pcl_version == '-1.7':
        # release
        # libreleases = ['pcl_common_release', 'pcl_features_release', 'pcl_filters_release', 'pcl_io_release', 'pcl_io_ply_release', 'pcl_kdtree_release', 'pcl_keypoints_release', 'pcl_octree_release', 'pcl_registration_release', 'pcl_sample_consensus_release', 'pcl_segmentation_release', 'pcl_search_release', 'pcl_surface_release', 'pcl_tracking_release', 'pcl_visualization_release', 'flann', 'flann_s']
        # release + vtk
        # libreleases = ['pcl_common_release', 'pcl_features_release', 'pcl_filters_release', 'pcl_io_release', 'pcl_io_ply_release', 'pcl_kdtree_release', 'pcl_keypoints_release', 'pcl_octree_release', 'pcl_outofcore_release', 'pcl_people_release', 'pcl_recognition_release','pcl_registration_release', 'pcl_sample_consensus_release', 'pcl_search_release', 'pcl_segmentation_release', 'pcl_surface_release', 'pcl_tracking_release', 'pcl_visualization_release', 'flann', 'flann_s', 'vtkInfovis', 'MapReduceMPI', 'vtkNetCDF', 'QVTK', 'vtkNetCDF_cxx', 'vtkRendering', 'vtkViews', 'vtkVolumeRendering', 'vtkWidgets', 'mpistubs', 'vtkalglib', 'vtkCharts', 'vtkexoIIc', 'vtkexpat', 'vtkCommon', 'vtkfreetype', 'vtkDICOMParser', 'vtkftgl', 'vtkFiltering', 'vtkhdf5', 'vtkjpeg', 'vtkGenericFiltering', 'vtklibxml2', 'vtkGeovis', 'vtkmetaio', 'vtkpng', 'vtkGraphics', 'vtkproj4', 'vtkHybrid', 'vtksqlite', 'vtksys', 'vtkIO', 'vtktiff', 'vtkImaging', 'vtkverdict', 'vtkzlib']
        # release + vtk6.2
        libreleases = ['pcl_common_release', 'pcl_features_release', 'pcl_filters_release', 'pcl_io_release', 'pcl_io_ply_release', 'pcl_kdtree_release', 'pcl_keypoints_release', 'pcl_octree_release', 'pcl_outofcore_release', 'pcl_people_release', 'pcl_recognition_release', 'pcl_registration_release', 'pcl_sample_consensus_release', 'pcl_search_release', 'pcl_segmentation_release', 'pcl_surface_release', 'pcl_tracking_release', 'pcl_visualization_release', 'flann', 'flann_s', 'vtkalglib-6.2', 'vtkChartsCore-6.2', 'vtkCommonColor-6.2', 'vtkCommonComputationalGeometry-6.2', 'vtkCommonCore-6.2', 'vtkCommonDataModel-6.2', 'vtkCommonExecutionModel-6.2', 'vtkCommonMath-6.2', 'vtkCommonMisc-6.2', 'vtkCommonSystem-6.2', 'vtkCommonTransforms-6.2', 'vtkDICOMParser-6.2', 'vtkDomainsChemistry-6.2', 'vtkexoIIc-6.2', 'vtkexpat-6.2', 'vtkFiltersAMR-6.2', 'vtkFiltersCore-6.2', 'vtkFiltersExtraction-6.2', 'vtkFiltersFlowPaths-6.2', 'vtkFiltersGeneral-6.2', 'vtkFiltersGeneric-6.2', 'vtkFiltersGeometry-6.2', 'vtkFiltersHybrid-6.2', 'vtkFiltersHyperTree-6.2', 'vtkFiltersImaging-6.2', 'vtkFiltersModeling-6.2', 'vtkFiltersParallel-6.2', 'vtkFiltersParallelImaging-6.2', 'vtkFiltersProgrammable-6.2', 'vtkFiltersSelection-6.2', 'vtkFiltersSMP-6.2', 'vtkFiltersSources-6.2', 'vtkFiltersStatistics-6.2', 'vtkFiltersTexture-6.2', 'vtkFiltersVerdict-6.2', 'vtkfreetype-6.2', 'vtkGeovisCore-6.2', 'vtkgl2ps-6.2', 'vtkhdf5-6.2', 'vtkhdf5_hl-6.2', 'vtkImagingColor-6.2', 'vtkImagingCore-6.2', 'vtkImagingFourier-6.2', 'vtkImagingGeneral-6.2', 'vtkImagingHybrid-6.2', 'vtkImagingMath-6.2', 'vtkImagingMorphological-6.2', 'vtkImagingSources-6.2', 'vtkImagingStatistics-6.2', 'vtkImagingStencil-6.2', 'vtkInfovisCore-6.2', 'vtkInfovisLayout-6.2', 'vtkInteractionImage-6.2', 'vtkInteractionStyle-6.2', 'vtkInteractionWidgets-6.2', 'vtkIOAMR-6.2', 'vtkIOCore-6.2', 'vtkIOEnSight-6.2', 'vtkIOExodus-6.2', 'vtkIOExport-6.2', 'vtkIOGeometry-6.2', 'vtkIOImage-6.2', 'vtkIOImport-6.2', 'vtkIOInfovis-6.2', 'vtkIOLegacy-6.2', 'vtkIOLSDyna-6.2', 'vtkIOMINC-6.2', 'vtkIOMovie-6.2', 'vtkIONetCDF-6.2', 'vtkIOParallel-6.2', 'vtkIOParallelXML-6.2', 'vtkIOPLY-6.2', 'vtkIOSQL-6.2', 'vtkIOVideo-6.2', 'vtkIOXML-6.2', 'vtkIOXMLParser-6.2', 'vtkjpeg-6.2', 'vtkjsoncpp-6.2', 'vtklibxml2-6.2', 'vtkmetaio-6.2', 'vtkNetCDF-6.2', 'vtkNetCDF_cxx-6.2', 'vtkoggtheora-6.2', 'vtkParallelCore-6.2', 'vtkpng-6.2', 'vtkproj4-6.2', 'vtkRenderingAnnotation-6.2', 'vtkRenderingContext2D-6.2', 'vtkRenderingContextOpenGL-6.2', 'vtkRenderingCore-6.2', 'vtkRenderingFreeType-6.2', 'vtkRenderingGL2PS-6.2', 'vtkRenderingImage-6.2', 'vtkRenderingLabel-6.2', 'vtkRenderingLIC-6.2', 'vtkRenderingLOD-6.2', 'vtkRenderingOpenGL-6.2', 'vtkRenderingVolume-6.2', 'vtkRenderingVolumeOpenGL-6.2', 'vtksqlite-6.2', 'vtksys-6.2', 'vtktiff-6.2', 'vtkverdict-6.2', 'vtkViewsContext2D-6.2', 'vtkViewsCore-6.2', 'vtkViewsInfovis-6.2', 'vtkzlib-6.2']

    elif pcl_version == '-1.8':
        # release
        # libreleases = ['pcl_common_release', 'pcl_features_release', 'pcl_filters_release', 'pcl_io_release', 'pcl_io_ply_release', 'pcl_kdtree_release', 'pcl_keypoints_release', 'pcl_octree_release', 'pcl_registration_release', 'pcl_sample_consensus_release', 'pcl_segmentation_release', 'pcl_search_release', 'pcl_surface_release', 'pcl_tracking_release', 'pcl_visualization_release', 'flann', 'flann_s']
        # libreleases = ['pcl_common_release', 'pcl_features_release', 'pcl_filters_release', 'pcl_io_release', 'pcl_io_ply_release', 'pcl_kdtree_release', 'pcl_keypoints_release', 'pcl_octree_release', 'pcl_registration_release', 'pcl_sample_consensus_release', 'pcl_segmentation_release', 'pcl_search_release', 'pcl_surface_release', 'pcl_tracking_release', 'pcl_visualization_release', 'flann', 'flann_s', 'vtkInfovis', 'MapReduceMPI', 'vtkNetCDF', 'QVTK', 'vtkNetCDF_cxx', 'vtkRendering', 'vtkViews', 'vtkVolumeRendering', 'vtkWidgets', 'mpistubs', 'vtkalglib', 'vtkCharts', 'vtkexoIIc', 'vtkexpat', 'vtkCommon', 'vtkfreetype', 'vtkDICOMParser', 'vtkftgl', 'vtkFiltering', 'vtkhdf5', 'vtkjpeg', 'vtkGenericFiltering', 'vtklibxml2', 'vtkGeovis', 'vtkmetaio', 'vtkpng', 'vtkGraphics', 'vtkproj4', 'vtkHybrid', 'vtksqlite', 'vtksys', 'vtkIO', 'vtktiff', 'vtkImaging', 'vtkverdict', 'vtkzlib']
        # release + vtk7.0
        libreleases = ['pcl_common_release', 'pcl_features_release', 'pcl_filters_release', 'pcl_io_release', 'pcl_io_ply_release', 'pcl_kdtree_release', 'pcl_keypoints_release', 'pcl_ml_release', 'pcl_octree_release', 'pcl_outofcore_release', 'pcl_people_release', 'pcl_recognition_release', 'pcl_registration_release', 'pcl_sample_consensus_release', 'pcl_search_release', 'pcl_segmentation_release', 'pcl_stereo_release', 'pcl_surface_release', 'pcl_tracking_release', 'pcl_visualization_release', 'flann', 'flann_s', 'vtkalglib-7.0', 'vtkChartsCore-7.0', 'vtkCommonColor-7.0', 'vtkCommonComputationalGeometry-7.0', 'vtkCommonCore-7.0', 'vtkCommonDataModel-7.0', 'vtkCommonExecutionModel-7.0', 'vtkCommonMath-7.0', 'vtkCommonMisc-7.0', 'vtkCommonSystem-7.0', 'vtkCommonTransforms-7.0', 'vtkDICOMParser-7.0', 'vtkDomainsChemistry-7.0', 'vtkexoIIc-7.0', 'vtkexpat-7.0', 'vtkFiltersAMR-7.0', 'vtkFiltersCore-7.0', 'vtkFiltersExtraction-7.0', 'vtkFiltersFlowPaths-7.0', 'vtkFiltersGeneral-7.0', 'vtkFiltersGeneric-7.0', 'vtkFiltersGeometry-7.0', 'vtkFiltersHybrid-7.0', 'vtkFiltersHyperTree-7.0', 'vtkFiltersImaging-7.0', 'vtkFiltersModeling-7.0', 'vtkFiltersParallel-7.0', 'vtkFiltersParallelImaging-7.0', 'vtkFiltersProgrammable-7.0', 'vtkFiltersSelection-7.0', 'vtkFiltersSMP-7.0', 'vtkFiltersSources-7.0', 'vtkFiltersStatistics-7.0', 'vtkFiltersTexture-7.0', 'vtkFiltersVerdict-7.0', 'vtkfreetype-7.0', 'vtkGeovisCore-7.0', 'vtkgl2ps-7.0', 'vtkhdf5-7.0', 'vtkhdf5_hl-7.0', 'vtkImagingColor-7.0', 'vtkImagingCore-7.0', 'vtkImagingFourier-7.0', 'vtkImagingGeneral-7.0', 'vtkImagingHybrid-7.0', 'vtkImagingMath-7.0', 'vtkImagingMorphological-7.0', 'vtkImagingSources-7.0', 'vtkImagingStatistics-7.0', 'vtkImagingStencil-7.0', 'vtkInfovisCore-7.0', 'vtkInfovisLayout-7.0', 'vtkInteractionImage-7.0', 'vtkInteractionStyle-7.0', 'vtkInteractionWidgets-7.0', 'vtkIOAMR-7.0', 'vtkIOCore-7.0', 'vtkIOEnSight-7.0', 'vtkIOExodus-7.0', 'vtkIOExport-7.0', 'vtkIOGeometry-7.0', 'vtkIOImage-7.0', 'vtkIOImport-7.0', 'vtkIOInfovis-7.0', 'vtkIOLegacy-7.0', 'vtkIOLSDyna-7.0', 'vtkIOMINC-7.0', 'vtkIOMovie-7.0', 'vtkIONetCDF-7.0', 'vtkIOParallel-7.0', 'vtkIOParallelXML-7.0', 'vtkIOPLY-7.0', 'vtkIOSQL-7.0', 'vtkIOVideo-7.0', 'vtkIOXML-7.0', 'vtkIOXMLParser-7.0', 'vtkjpeg-7.0', 'vtkjsoncpp-7.0', 'vtklibxml2-7.0', 'vtkmetaio-7.0', 'vtkNetCDF-7.0', 'vtkNetCDF_cxx-7.0', 'vtkoggtheora-7.0', 'vtkParallelCore-7.0', 'vtkpng-7.0', 'vtkproj4-7.0', 'vtkRenderingAnnotation-7.0', 'vtkRenderingContext2D-7.0', 'vtkRenderingContextOpenGL-7.0', 'vtkRenderingCore-7.0', 'vtkRenderingFreeType-7.0', 'vtkRenderingGL2PS-7.0', 'vtkRenderingImage-7.0', 'vtkRenderingLabel-7.0', 'vtkRenderingLIC-7.0', 'vtkRenderingLOD-7.0', 'vtkRenderingOpenGL-7.0', 'vtkRenderingVolume-7.0', 'vtkRenderingVolumeOpenGL-7.0', 'vtksqlite-7.0', 'vtksys-7.0', 'vtktiff-7.0', 'vtkverdict-7.0', 'vtkViewsContext2D-7.0', 'vtkViewsCore-7.0', 'vtkViewsInfovis-7.0', 'vtkzlib-7.0']
        
        # add boost
        # dynamic lib
        # libreleases = ['pcl_common_release', 'pcl_features_release', 'pcl_filters_release', 'pcl_io_release', 'pcl_io_ply_release', 'pcl_kdtree_release', 'pcl_keypoints_release', 'pcl_octree_release', 'pcl_registration_release', 'pcl_sample_consensus_release', 'pcl_segmentation_release', 'pcl_search_release', 'pcl_surface_release', 'pcl_tracking_release', 'pcl_visualization_release', 'flann', 'flann_s', 'boost_date_time-vc100-mt-1_47', 'boost_filesystem-vc100-mt-1_49', 'boost_graph-vc100-mt-1_49', 'boost_graph_parallel-vc100-mt-1_49', 'boost_iostreams-vc100-mt-1_49', 'boost_locale-vc100-mt-1_49', 'boost_math_c99-vc100-mt-1_49', 'boost_math_c99f-vc100-mt-1_49', 'boost_math_tr1-vc100-mt-1_49', 'boost_math_tr1f-vc100-mt-1_49', 'boost_mpi-vc100-mt-1_49', 'boost_prg_exec_monitor-vc100-mt-1_49', 'boost_program_options-vc100-mt-1_49', 'boost_random-vc100-mt-1_49', 'boost_regex-vc100-mt-1_49', 'boost_serialization-vc100-mt-1_49', 'boost_signals-vc100-mt-1_49', 'boost_system-vc100-mt-1_49', 'boost_thread-vc100-mt-1_49', 'boost_timer-vc100-mt-1_49', 'boost_unit_test_framework-vc100-mt-1_49', 'boost_wave-vc100-mt-1_49', 'boost_wserialization-vc100-mt-1_49']
        # static lib
        # libreleases = ['pcl_common_release', 'pcl_features_release', 'pcl_filters_release', 'pcl_io_release', 'pcl_io_ply_release', 'pcl_kdtree_release', 'pcl_keypoints_release', 'pcl_octree_release', 'pcl_registration_release', 'pcl_sample_consensus_release', 'pcl_segmentation_release', 'pcl_search_release', 'pcl_surface_release', 'pcl_tracking_release', 'pcl_visualization_release', 'flann', 'flann_s', 'libboost_date_time-vc100-mt-1_49', 'libboost_filesystem-vc100-mt-1_49', 'libboost_graph_parallel-vc100-mt-1_49', 'libboost_iostreams-vc100-mt-1_49', 'libboost_locale-vc100-mt-1_49', 'libboost_math_c99-vc100-mt-1_49', 'libboost_math_c99f-vc100-mt-1_49', 'libboost_math_tr1-vc100-mt-1_49', 'libboost_math_tr1f-vc100-mt-1_49', 'libboost_mpi-vc100-mt-1_49', 'libboost_prg_exec_monitor-vc100-mt-1_49', 'libboost_program_options-vc100-mt-1_49', 'libboost_random-vc100-mt-1_49', 'libboost_regex-vc100-mt-1_49', 'libboost_serialization-vc100-mt-1_49', 'libboost_signals-vc100-mt-1_49', 'libboost_system-vc100-mt-1_49', 'libboost_test_exec_monitor-vc100-mt-1_49', 'libboost_thread-vc100-mt-1_49', 'libboost_timer-vc100-mt-1_49', 'libboost_unit_test_framework-vc100-mt-1_49', 'libboost_wave-vc100-mt-1_49', 'libboost_wserialization-vc100-mt-1_49']
    elif pcl_version == '-1.9':
        # release + vtk7.0?
        libreleases = ['pcl_common_release', 'pcl_features_release', 'pcl_filters_release', 'pcl_io_release', 'pcl_io_ply_release', 'pcl_kdtree_release', 'pcl_keypoints_release', 'pcl_ml_release', 'pcl_octree_release', 'pcl_outofcore_release', 'pcl_people_release', 'pcl_recognition_release', 'pcl_registration_release', 'pcl_sample_consensus_release', 'pcl_search_release', 'pcl_segmentation_release', 'pcl_stereo_release', 'pcl_surface_release', 'pcl_tracking_release', 'pcl_visualization_release', 'flann', 'flann_s', 'vtkalglib-7.0', 'vtkChartsCore-7.0', 'vtkCommonColor-7.0', 'vtkCommonComputationalGeometry-7.0', 'vtkCommonCore-7.0', 'vtkCommonDataModel-7.0', 'vtkCommonExecutionModel-7.0', 'vtkCommonMath-7.0', 'vtkCommonMisc-7.0', 'vtkCommonSystem-7.0', 'vtkCommonTransforms-7.0', 'vtkDICOMParser-7.0', 'vtkDomainsChemistry-7.0', 'vtkexoIIc-7.0', 'vtkexpat-7.0', 'vtkFiltersAMR-7.0', 'vtkFiltersCore-7.0', 'vtkFiltersExtraction-7.0', 'vtkFiltersFlowPaths-7.0', 'vtkFiltersGeneral-7.0', 'vtkFiltersGeneric-7.0', 'vtkFiltersGeometry-7.0', 'vtkFiltersHybrid-7.0', 'vtkFiltersHyperTree-7.0', 'vtkFiltersImaging-7.0', 'vtkFiltersModeling-7.0', 'vtkFiltersParallel-7.0', 'vtkFiltersParallelImaging-7.0', 'vtkFiltersProgrammable-7.0', 'vtkFiltersSelection-7.0', 'vtkFiltersSMP-7.0', 'vtkFiltersSources-7.0', 'vtkFiltersStatistics-7.0', 'vtkFiltersTexture-7.0', 'vtkFiltersVerdict-7.0', 'vtkfreetype-7.0', 'vtkGeovisCore-7.0', 'vtkgl2ps-7.0', 'vtkhdf5-7.0', 'vtkhdf5_hl-7.0', 'vtkImagingColor-7.0', 'vtkImagingCore-7.0', 'vtkImagingFourier-7.0', 'vtkImagingGeneral-7.0', 'vtkImagingHybrid-7.0', 'vtkImagingMath-7.0', 'vtkImagingMorphological-7.0', 'vtkImagingSources-7.0', 'vtkImagingStatistics-7.0', 'vtkImagingStencil-7.0', 'vtkInfovisCore-7.0', 'vtkInfovisLayout-7.0', 'vtkInteractionImage-7.0', 'vtkInteractionStyle-7.0', 'vtkInteractionWidgets-7.0', 'vtkIOAMR-7.0', 'vtkIOCore-7.0', 'vtkIOEnSight-7.0', 'vtkIOExodus-7.0', 'vtkIOExport-7.0', 'vtkIOGeometry-7.0', 'vtkIOImage-7.0', 'vtkIOImport-7.0', 'vtkIOInfovis-7.0', 'vtkIOLegacy-7.0', 'vtkIOLSDyna-7.0', 'vtkIOMINC-7.0', 'vtkIOMovie-7.0', 'vtkIONetCDF-7.0', 'vtkIOParallel-7.0', 'vtkIOParallelXML-7.0', 'vtkIOPLY-7.0', 'vtkIOSQL-7.0', 'vtkIOVideo-7.0', 'vtkIOXML-7.0', 'vtkIOXMLParser-7.0', 'vtkjpeg-7.0', 'vtkjsoncpp-7.0', 'vtklibxml2-7.0', 'vtkmetaio-7.0', 'vtkNetCDF-7.0', 'vtkNetCDF_cxx-7.0', 'vtkoggtheora-7.0', 'vtkParallelCore-7.0', 'vtkpng-7.0', 'vtkproj4-7.0', 'vtkRenderingAnnotation-7.0', 'vtkRenderingContext2D-7.0', 'vtkRenderingContextOpenGL-7.0', 'vtkRenderingCore-7.0', 'vtkRenderingFreeType-7.0', 'vtkRenderingGL2PS-7.0', 'vtkRenderingImage-7.0', 'vtkRenderingLabel-7.0', 'vtkRenderingLIC-7.0', 'vtkRenderingLOD-7.0', 'vtkRenderingOpenGL-7.0', 'vtkRenderingVolume-7.0', 'vtkRenderingVolumeOpenGL-7.0', 'vtksqlite-7.0', 'vtksys-7.0', 'vtktiff-7.0', 'vtkverdict-7.0', 'vtkViewsContext2D-7.0', 'vtkViewsCore-7.0', 'vtkViewsInfovis-7.0', 'vtkzlib-7.0']
    else:
        libreleases = []
    
    for librelease in libreleases:
        ext_args['libraries'].append(librelease)
    
    # Note : 
    # vtk Version setting
    
    # use vtk need library(Windows base library)
    # http://public.kitware.com/pipermail/vtkusers/2008-July/047291.html
    win_libreleases = ['kernel32', 'user32', 'gdi32', 'winspool', 'comdlg32', 'advapi32', 'shell32', 'ole32', 'oleaut32', 'uuid', 'odbc32', 'odbccp32']
    for win_librelease in win_libreleases:
        ext_args['libraries'].append(win_librelease)
    
    # http://www.pcl-users.org/error-in-use-PCLVisualizer-td3719235.html
    # Download MSSDKs
    # http://msdn.microsoft.com/en-us/windows/bb980924.aspx
    # 
    # http://stackoverflow.com/questions/1236670/how-to-make-opengl-apps-in-64-bits-windows
    # C:\Program Files (x86)\Microsoft SDKs\Windows\7.0\Lib\x64\OpenGL32.lib
    # C:\Program Files (x86)\Microsoft SDKs\Windows\v7.0A\Lib\x64\OpenGL32.lib
    # Add OpenGL32 .h/.lib
    if pcl_version == '-1.7':
        if is_64bits == True:
            win_opengl_libdirs = ['C:\\Program Files (x86)\\Microsoft SDKs\\Windows\\v8.0A\\Lib\\x64']
        else:
            win_opengl_libdirs = ['C:\\Program Files (x86)\\Microsoft SDKs\\Windows\\v8.0A\\Lib\\win32']
    elif pcl_version == '-1.8':
        if is_64bits == True:
            # win_opengl_libdirs = ['C:\\Program Files (x86)\\Microsoft SDKs\\Windows\\v8.1A\\Lib\\x64']
            win_opengl_libdirs = []
        else:
            win_opengl_libdirs = ['C:\\Program Files (x86)\\Microsoft SDKs\\Windows\\v8.1A\\Lib\\win32']
    elif pcl_version == '-1.9':
        if is_64bits == True:
            win_opengl_libdirs = ['C:\\Program Files (x86)\\Microsoft SDKs\\Windows\\v8.1A\\Lib\\x64']
        else:
            win_opengl_libdirs = ['C:\\Program Files (x86)\\Microsoft SDKs\\Windows\\v8.1A\\Lib\\win32']
    else:
        pass
    
    for lib_dir in win_opengl_libdirs:
        ext_args['library_dirs'].append(lib_dir)
    
    win_opengl_libreleases = ['OpenGL32']
    for opengl_librelease in win_opengl_libreleases:
        ext_args['libraries'].append(opengl_librelease)
    
    # use OpenNI
    # use OpenNI2
    # add environment PATH : pcl/bin, OpenNI2/Tools
    
    # use CUDA?
    # CUDA_PATH
    
    # ext_args['define_macros'].append(('EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET', '1'))
    # define_macros=[('BOOST_NO_EXCEPTIONS', 'None')],
    # debugs = [('EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET', '1'), ('BOOST_NO_EXCEPTIONS', 'None')]
    # _CRT_SECURE_NO_WARNINGS : windows cutr warning no view
    defines = [('EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET', '1'), ('_CRT_SECURE_NO_WARNINGS', '1')]
    for define in defines:
        ext_args['define_macros'].append(define)
    
    # ext_args['extra_compile_args'].append('/DWIN32')
    # ext_args['extra_compile_args'].append('/D_WINDOWS')
    # ext_args['extra_compile_args'].append('/W3')
    # ext_args['extra_compile_args'].append('/GR')
    ext_args['extra_compile_args'].append('/EHsc')
    # FW: Link time errors in RangeImage (with /clr)
    # http://www.pcl-users.org/FW-Link-time-errors-in-RangeImage-with-clr-td3581422.html
    # ext_args['extra_compile_args'].append('/clr:nostdlib')
    
    # NG
    # ext_args['extra_compile_args'].append('/NODEFAULTLIB:msvcrtd')
    # https://blogs.msdn.microsoft.com/vcblog/2015/03/03/introducing-the-universal-crt/
    # default args
    # ext_args['extra_compile_args'].append('/MD')
    # custom
    # ext_args['extra_compile_args'].append('/MDd')
    # ext_args['extra_compile_args'].append('/MTd')
    # ext_args['extra_compile_args'].append('/MT')
    
    # include_dirs=[pcl_root + '\\include\\pcl' + pcl_version, pcl_root + '\\3rdParty\\Eigen\\include', pcl_root + '\\3rdParty\\Boost\\include', pcl_root + '\\3rdParty\\FLANN\include', 'C:\\Anaconda2\\envs\\my_env\\Lib\\site-packages\\numpy\\core\\include'],
    # library_dirs=[pcl_root + '\\lib', pcl_root + '\\3rdParty\\Boost\\lib', pcl_root + '\\3rdParty\\FLANN\\lib'],
    # libraries=["pcl_apps_debug", "pcl_common_debug", "pcl_features_debug", "pcl_filters_debug", "pcl_io_debug", "pcl_io_ply_debug", "pcl_kdtree_debug", "pcl_keypoints_debug", "pcl_octree_debug", "pcl_registration_debug", "pcl_sample_consensus_debug", "pcl_segmentation_debug", "pcl_search_debug", "pcl_surface_debug", "pcl_tracking_debug", "pcl_visualization_debug", "flann-gd", "flann_s-gd", "boost_chrono-vc100-mt-1_49", "boost_date_time-vc100-mt-1_49", "boost_filesystem-vc100-mt-1_49", "boost_graph-vc100-mt-1_49", "boost_graph_parallel-vc100-mt-1_49", "boost_iostreams-vc100-mt-1_49", "boost_locale-vc100-mt-1_49", "boost_math_c99-vc100-mt-1_49", "boost_math_c99f-vc100-mt-1_49", "boost_math_tr1-vc100-mt-1_49", "boost_math_tr1f-vc100-mt-1_49", "boost_mpi-vc100-mt-1_49", "boost_prg_exec_monitor-vc100-mt-1_49", "boost_program_options-vc100-mt-1_49", "boost_random-vc100-mt-1_49", "boost_regex-vc100-mt-1_49", "boost_serialization-vc100-mt-1_49", "boost_signals-vc100-mt-1_49", "boost_system-vc100-mt-1_49", "boost_thread-vc100-mt-1_49", "boost_timer-vc100-mt-1_49", "boost_unit_test_framework-vc100-mt-1_49", "boost_wave-vc100-mt-1_49", "boost_wserialization-vc100-mt-1_49", "libboost_chrono-vc100-mt-1_49", "libboost_date_time-vc100-mt-1_49", "libboost_filesystem-vc100-mt-1_49", "libboost_graph_parallel-vc100-mt-1_49", "libboost_iostreams-vc100-mt-1_49", "libboost_locale-vc100-mt-1_49", "libboost_math_c99-vc100-mt-1_49", "libboost_math_c99f-vc100-mt-1_49", "libboost_math_tr1-vc100-mt-1_49", "libboost_math_tr1f-vc100-mt-1_49", "libboost_mpi-vc100-mt-1_49", "libboost_prg_exec_monitor-vc100-mt-1_49", "libboost_program_options-vc100-mt-1_49", "libboost_random-vc100-mt-1_49", "libboost_regex-vc100-mt-1_49", "libboost_serialization-vc100-mt-1_49", "libboost_signals-vc100-mt-1_49", "libboost_system-vc100-mt-1_49", "libboost_test_exec_monitor-vc100-mt-1_49", "libboost_thread-vc100-mt-1_49", "libboost_timer-vc100-mt-1_49", "libboost_unit_test_framework-vc100-mt-1_49", "libboost_wave-vc100-mt-1_49", "libboost_wserialization-vc100-mt-1_49"],
    ## define_macros=[('BOOST_NO_EXCEPTIONS', 'None')],
    # define_macros=[('EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET', '1')],
    # extra_compile_args=["/EHsc"],
    
    print(ext_args)
    
    if pcl_version == '-1.7':
        module = [Extension("pcl._pcl", ["pcl/_pcl_172.pyx", "pcl/minipcl.cpp", "pcl/ProjectInliers.cpp"], language = "c++", **ext_args),
                  Extension("pcl.pcl_visualization", ["pcl/pcl_visualization.pyx"], language="c++", **ext_args),
                  # Extension("pcl.pcl_grabber", ["pcl/pcl_grabber.pyx", "pcl/grabber_callback.cpp"], language="c++", **ext_args),
                  # debug
                  # gdb_debug=True,
                  ]
    elif pcl_version == '-1.8':
        module = [Extension("pcl._pcl", ["pcl/_pcl_180.pyx", "pcl/minipcl.cpp", "pcl/ProjectInliers.cpp"], language = "c++", **ext_args),
                  Extension("pcl.pcl_visualization", ["pcl/pcl_visualization.pyx"], language="c++", **ext_args),
                  # Extension("pcl.pcl_grabber", ["pcl/pcl_grabber.pyx", "pcl/grabber_callback.cpp"], language="c++", **ext_args),
                  # debug
                  # gdb_debug=True,
                  ]
    elif pcl_version == '-1.9':
        module = [Extension("pcl._pcl", ["pcl/_pcl_172.pyx", "pcl/minipcl.cpp", "pcl/ProjectInliers.cpp"], language = "c++", **ext_args),
                  Extension("pcl.pcl_visualization", ["pcl/pcl_visualization.pyx"], language="c++", **ext_args),
                  # Extension("pcl.pcl_grabber", ["pcl/pcl_grabber.pyx", "pcl/grabber_callback.cpp"], language="c++", **ext_args),
                  # debug
                  # gdb_debug=True,
                  ]
    else:
        print('no pcl install or pkg-config missed.')
        sys.exit(1)
else:
    # Not 'Windows'
    if platform.system() == "Darwin":
        os.environ['ARCHFLAGS'] = ''
    
    # Try to find PCL. XXX we should only do this when trying to build or install.
    PCL_SUPPORTED = ["-1.9", "-1.8", "-1.7", ""]    # in order of preference
    
    for pcl_version in PCL_SUPPORTED:
        if subprocess.call(['pkg-config', 'pcl_common%s' % pcl_version]) == 0:
            break
    else:
        print("%s: error: cannot find PCL, tried" % sys.argv[0], file=sys.stderr)
        for version in PCL_SUPPORTED:
            print('    pkg-config pcl_common%s' % version, file=sys.stderr)
        sys.exit(1)
    
    # Find build/link options for PCL using pkg-config.
    pcl_libs = ["common", "features", "filters", "io", "kdtree", "octree",
                "registration", "sample_consensus", "search", "segmentation",
                "surface", "tracking", "visualization"]
    pcl_libs = ["pcl_%s%s" % (lib, pcl_version) for lib in pcl_libs]
    
    ext_args = defaultdict(list)
    ext_args['include_dirs'].append(numpy.get_include())
    
    def pkgconfig(flag):
        # Equivalent in Python 2.7 (but not 2.6):
        # subprocess.check_output(['pkg-config', flag] + pcl_libs).split()
        p = subprocess.Popen(['pkg-config', flag] + pcl_libs, stdout=subprocess.PIPE)
        stdout, _ = p.communicate()
        # Assume no evil spaces in filenames; unsure how pkg-config would
        # handle those, anyway.
        # decode() is required in Python 3. TODO how do know the encoding?
        return stdout.decode().split()
    
    for flag in pkgconfig('--cflags-only-I'):
        ext_args['include_dirs'].append(flag[2:])
    
    # OpenNI?
    # "-I/usr/include/openni"
    # "-I/usr/include/openni"
    # /usr/include/ni
    ext_args['include_dirs'].append('/usr/include/ni')
    # ext_args['library_dirs'].append()
    # ext_args['libraries'].append()
    
    for flag in pkgconfig('--cflags-only-other'):
        if flag.startswith('-D'):
            macro, value = flag[2:].split('=', 1)
            ext_args['define_macros'].append((macro, value))
        else:
            ext_args['extra_compile_args'].append(flag)
    
    for flag in pkgconfig('--libs-only-l'):
        if flag == "-lflann_cpp-gd":
            print("skipping -lflann_cpp-gd (see https://github.com/strawlab/python-pcl/issues/29")
            continue
        ext_args['libraries'].append(flag[2:])
    
    for flag in pkgconfig('--libs-only-L'):
        ext_args['library_dirs'].append(flag[2:])
    
    for flag in pkgconfig('--libs-only-other'):
        ext_args['extra_link_args'].append(flag)
    
    # grabber?
    # -lboost_system
    ext_args['extra_link_args'].append('-lboost_system')
    # ext_args['extra_link_args'].append('-lboost_bind')
    
    # Fix compile error on Ubuntu 12.04 (e.g., Travis-CI).
    ext_args['define_macros'].append(
        ("EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET", "1"))
    
    if pcl_version == '-1.7':
        module = [Extension("pcl._pcl", ["pcl/_pcl_172.pyx", "pcl/minipcl.cpp", "pcl/ProjectInliers.cpp"], language = "c++", **ext_args),
                  # Extension("pcl.pcl_visualization", ["pcl/pcl_visualization.pyx"], language="c++", **ext_args),
                  # Extension("pcl.pcl_grabber", ["pcl/pcl_grabber.pyx", "pcl/grabber_callback.cpp"], language="c++", **ext_args),
                  # debug
                  # gdb_debug=True,
                  ]
    elif pcl_version == '-1.8':
        module = [Extension("pcl._pcl", ["pcl/_pcl_180.pyx", "pcl/minipcl.cpp", "pcl/ProjectInliers.cpp"], language = "c++", **ext_args),
                           # Extension("pcl.pcl_visualization", ["pcl/pcl_visualization.pyx"], language="c++", **ext_args),
                           # Extension("pcl.pcl_grabber", ["pcl/pcl_grabber.pyx", "pcl/grabber_callback.cpp"], language="c++", **ext_args),
                           # debug
                           # gdb_debug=True,
                          ]
    elif pcl_version == '-1.9':
        module = [Extension("pcl._pcl", ["pcl/_pcl_190.pyx", "pcl/minipcl.cpp", "pcl/ProjectInliers.cpp"], language = "c++", **ext_args),
                           # Extension("pcl.pcl_visualization", ["pcl/pcl_visualization.pyx"], language="c++", **ext_args),
                           # Extension("pcl.pcl_grabber", ["pcl/pcl_grabber.pyx", "pcl/grabber_callback.cpp"], language="c++", **ext_args),
                           # debug
                           # gdb_debug=True,
                          ]
    else:
        print('no pcl install or pkg-config missed.')
        sys.exit(1)
    

setup(name='python-pcl',
    description='pcl wrapper',
    url='http://github.com/strawlab/python-pcl',
    version='0.2',
    author='John Stowers',
    author_email='john.stowers@gmail.com',
    license='BSD',
    packages=["pcl"],
    zip_safe=False,
    setup_requires=setup_requires,
    install_requires=install_requires,
    tests_require=['mock', 'nose'],
    ext_modules=module,
    cmdclass={'build_ext': build_ext}
    )
