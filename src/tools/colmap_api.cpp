//
// Created by maxim on 29.04.2020.
//
#include "base/similarity_transform.h"
#include "controllers/automatic_reconstruction.h"
#include "controllers/bundle_adjustment.h"
#include "controllers/hierarchical_mapper.h"
#include "estimators/coordinate_frame.h"
#include "feature/extraction.h"
#include "feature/matching.h"
#include "feature/utils.h"
#include "mvs/meshing.h"
#include "mvs/patch_match.h"
#include "retrieval/visual_index.h"
#include "ui/main_window.h"
#include "util/opengl_utils.h"
#include "util/version.h"

#include "colmap_api.h"

namespace colmap {

   #ifdef CUDA_ENABLED
   const bool kUseOpenGL = false;
   #else
   const bool kUseOpenGL = true;
   #endif

   bool VerifyCameraParams(const std::string& camera_model, const std::string& params) {
        if (!ExistsCameraModelWithName(camera_model)) {
            std::cerr << "ERROR: Camera model does not exist" << std::endl;
            return false;
        }
        const std::vector<double> camera_params = CSVToVector<double>(params);
        const int camera_model_id = CameraModelNameToId(camera_model);

        if (camera_params.size() > 0 && !CameraModelVerifyParams(camera_model_id, camera_params)) {
            std::cerr << "ERROR: Invalid camera parameters" << std::endl;
            return false;
        }
        return true;
   }

    int ExtractFeatures(const std::string& project_path) {
        std::string features_image_list_path;
        OptionManager options;
        options.AddDatabaseOptions();
        options.AddImageOptions();
        options.AddDefaultOption("features_image_list_path", &features_image_list_path);
        options.AddExtractionOptions();
        options.Read(project_path);
        ImageReaderOptions reader_options = *options.image_reader;
        reader_options.database_path = *options.database_path;
        reader_options.image_path = *options.image_path;

        if (!features_image_list_path.empty()) {
            reader_options.image_list = ReadTextFileLines(features_image_list_path);
            if (reader_options.image_list.empty()) {
                return EXIT_SUCCESS;
            }
        }

        if (!ExistsCameraModelWithName(options.image_reader->camera_model)) {
            std::cerr << "ERROR: Camera model does not exist" << std::endl;
        }

        if (!VerifyCameraParams(options.image_reader->camera_model,
                                options.image_reader->camera_params)) {
            return EXIT_FAILURE;
        }

        SiftFeatureExtractor feature_extractor(reader_options,
                                               *options.sift_extraction);

        if (options.sift_extraction->use_gpu && kUseOpenGL) {
            RunThreadWithOpenGLContext(&feature_extractor);
        } else {
            feature_extractor.Start();
            feature_extractor.Wait();
        }

        return EXIT_SUCCESS;
    }

    int MatchFeaturesExhaustively(const std::string& project_path) {
        OptionManager options;
        options.AddDatabaseOptions();
        options.AddExhaustiveMatchingOptions();
        options.Read(project_path);

        ExhaustiveFeatureMatcher feature_matcher(*options.exhaustive_matching,
                                                 *options.sift_matching,
                                                 *options.database_path);

        if (options.sift_matching->use_gpu && kUseOpenGL) {
            RunThreadWithOpenGLContext(&feature_matcher);
        } else {
            feature_matcher.Start();
            feature_matcher.Wait();
        }

        return EXIT_SUCCESS;
    }

    int ReconstructSparse(const std::string& project_path) {
        std::string mapper_input_path;
        std::string mapper_output_path;
        std::string mapper_image_list_path;

        OptionManager options;
        options.AddDatabaseOptions();
        options.AddImageOptions();
        options.AddDefaultOption("mapper_input_path", &mapper_input_path);
        options.AddRequiredOption("mapper_output_path", &mapper_output_path);
        options.AddDefaultOption("mapper_image_list_path", &mapper_image_list_path);
        options.AddMapperOptions();
        options.Read(project_path);

        if (!ExistsDir(mapper_output_path)) {
            std::cerr << "ERROR: `output_path` is not a directory." << std::endl;
            return EXIT_FAILURE;
        }

        if (!mapper_image_list_path.empty()) {
            const auto image_names = ReadTextFileLines(mapper_image_list_path);
            options.mapper->image_names =
                std::unordered_set<std::string>(image_names.begin(), image_names.end());
        }

        ReconstructionManager reconstruction_manager;
        if (mapper_input_path != "") {
            if (!ExistsDir(mapper_input_path)) {
                std::cerr << "ERROR: `input_path` is not a directory." << std::endl;
                return EXIT_FAILURE;
            }
            reconstruction_manager.Read(mapper_input_path);
        }

        IncrementalMapperController mapper(options.mapper.get(), *options.image_path,
                                            *options.database_path,
                                            &reconstruction_manager);

        size_t prev_num_reconstructions = 0;
        if (mapper_input_path == "") {
            mapper.AddCallback(
                IncrementalMapperController::LAST_IMAGE_REG_CALLBACK, [&]() {
                if (reconstruction_manager.Size() > prev_num_reconstructions) {
                    const std::string reconstruction_path = JoinPaths(
                        mapper_output_path, std::to_string(prev_num_reconstructions));
                    const auto& reconstruction =
                        reconstruction_manager.Get(prev_num_reconstructions);
                    CreateDirIfNotExists(reconstruction_path);
                    reconstruction.Write(reconstruction_path);
                    options.Write(JoinPaths(reconstruction_path, "project.ini"));
                    prev_num_reconstructions = reconstruction_manager.Size();
                }
                });
        }

        mapper.Start();
        mapper.Wait();

        if (mapper_input_path != "" && reconstruction_manager.Size() > 0) {
            reconstruction_manager.Get(0).Write(mapper_output_path);
        }

        return EXIT_SUCCESS;
    }

    int ConvertModel(const std::string& project_path) {
        std::string converter_input_path;
        std::string converter_output_path;
        std::string converter_output_type;

        OptionManager options;
        options.AddRequiredOption("converter_input_path", &converter_input_path);
        options.AddRequiredOption("converter_output_path", &converter_output_path);
        options.AddRequiredOption("converter_output_type", &converter_output_type,
                                    "{BIN, TXT, NVM, Bundler, VRML, PLY}");
        options.Read(project_path);

        Reconstruction reconstruction;
        reconstruction.Read(converter_input_path);

        StringToLower(&converter_output_type);
        if (converter_output_type == "bin") {
            reconstruction.WriteBinary(converter_output_path);
        } else if (converter_output_type == "txt") {
            reconstruction.WriteText(converter_output_path);
        } else if (converter_output_type == "nvm") {
            reconstruction.ExportNVM(converter_output_path);
        } else if (converter_output_type == "bundler") {
            reconstruction.ExportBundler(converter_output_path + ".bundle.out",
                                         converter_output_path + ".list.txt");
        } else if (converter_output_type == "ply") {
            reconstruction.ExportPLY(converter_output_path);
        } else if (converter_output_type == "vrml") {
            const auto base_path = converter_output_path.substr(0, converter_output_path.find_last_of("."));
            reconstruction.ExportVRML(base_path + ".images.wrl",
                                      base_path + ".points3D.wrl", 1,
                                      Eigen::Vector3d(1, 0, 0));
        } else {
            std::cerr << "ERROR: Invalid `output_type`" << std::endl;
            return EXIT_FAILURE;
        }

        return EXIT_SUCCESS;
        }

    int UndistortImages(const std::string& project_path) {
        std::string model_input_path;
        std::string undistorter_output_path;
        std::string undistorter_output_type = "COLMAP";

        UndistortCameraOptions undistort_camera_options;

        OptionManager options;
        options.AddImageOptions();
        options.AddRequiredOption("model_input_path", &model_input_path);
        options.AddRequiredOption("undistorter_output_path", &undistorter_output_path);
        options.AddDefaultOption("undistorter_output_type", &undistorter_output_type,
                                "{COLMAP, PMVS, CMP-MVS}");
        options.AddDefaultOption("blank_pixels",
                                 &undistort_camera_options.blank_pixels);
        options.AddDefaultOption("min_scale", &undistort_camera_options.min_scale);
        options.AddDefaultOption("max_scale", &undistort_camera_options.max_scale);
        options.AddDefaultOption("max_image_size",
                                 &undistort_camera_options.max_image_size);
        options.AddDefaultOption("roi_min_x", &undistort_camera_options.roi_min_x);
        options.AddDefaultOption("roi_min_y", &undistort_camera_options.roi_min_y);
        options.AddDefaultOption("roi_max_x", &undistort_camera_options.roi_max_x);
        options.AddDefaultOption("roi_max_y", &undistort_camera_options.roi_max_y);
        options.Read(project_path);

        CreateDirIfNotExists(undistorter_output_path);

        Reconstruction reconstruction;
        reconstruction.Read(model_input_path);

        std::unique_ptr<Thread> undistorter;
        if (undistorter_output_type == "COLMAP") {
            undistorter.reset(new COLMAPUndistorter(undistort_camera_options,
                                                    reconstruction, *options.image_path,
                                                    undistorter_output_path));
        } else if (undistorter_output_type == "PMVS") {
            undistorter.reset(new PMVSUndistorter(undistort_camera_options,
                                                reconstruction, *options.image_path,
                                                undistorter_output_path));
        } else if (undistorter_output_type == "CMP-MVS") {
            undistorter.reset(new CMPMVSUndistorter(undistort_camera_options,
                                                    reconstruction, *options.image_path,
                                                    undistorter_output_path));
        } else {
            std::cerr << "ERROR: Invalid `output_type` - supported values are "
                        "{'COLMAP', 'PMVS', 'CMP-MVS'}."
                    << std::endl;
            return EXIT_FAILURE;
        }

        undistorter->Start();
        undistorter->Wait();

        return EXIT_SUCCESS;
    }

    int CreateDatabase(const std::string& project_path) {
        OptionManager options;
        options.AddDatabaseOptions();
        options.Read(project_path);

        Database database(*options.database_path);

        return EXIT_SUCCESS;
    }

    int PatchMatchStereo(const std::string& project_path) {
        #ifndef CUDA_ENABLED
        std::cerr << "ERROR: Dense stereo reconstruction requires CUDA, which is not "
                    "available on your system."
                    << std::endl;
        return EXIT_FAILURE;
        #else   // CUDA_ENABLED
        std::string dense_workspace_path;
        std::string dense_workspace_format = "COLMAP";
        std::string pmvs_option_name = "option-all";

        OptionManager options;
        options.AddRequiredOption(
            "dense_workspace_path", &dense_workspace_path,
            "Path to the folder containing the undistorted images");
        options.AddDefaultOption("dense_workspace_format", &dense_workspace_format,
                                "{COLMAP, PMVS}");
        options.AddDefaultOption("pmvs_option_name", &pmvs_option_name);
        options.AddPatchMatchStereoOptions();
        options.Read(project_path);

        StringToLower(&dense_workspace_format);
        if (dense_workspace_format != "colmap" && dense_workspace_format != "pmvs") {
            std::cout << "ERROR: Invalid `workspace_format` - supported values are "
                        "'COLMAP' or 'PMVS'."
                    << std::endl;
            return EXIT_FAILURE;
        }

        mvs::PatchMatchController controller(*options.patch_match_stereo,
                                            dense_workspace_path, dense_workspace_format,
                                            pmvs_option_name);

        controller.Start();
        controller.Wait();

        return EXIT_SUCCESS;
        #endif  // CUDA_ENABLED
    }

    int StereoFuser(const std::string& project_path) {
        std::string dense_workspace_path;
        std::string input_type = "geometric";
        std::string workspace_format = "COLMAP";
        std::string pmvs_option_name = "option-all";
        std::string dense_output_path;

        OptionManager options;
        options.AddRequiredOption("dense_workspace_path", &dense_workspace_path);
        options.AddDefaultOption("workspace_format", &workspace_format,
                                "{COLMAP, PMVS}");
        options.AddDefaultOption("pmvs_option_name", &pmvs_option_name);
        options.AddDefaultOption("input_type", &input_type,
                                "{photometric, geometric}");
        options.AddRequiredOption("dense_output_path", &dense_output_path);
        options.AddStereoFusionOptions();
        options.Read(project_path); 
        StringToLower(&workspace_format);
        if (workspace_format != "colmap" && workspace_format != "pmvs") {
            std::cout << "ERROR: Invalid `workspace_format` - supported values are "
                        "'COLMAP' or 'PMVS'."
                    << std::endl;
            return EXIT_FAILURE;
        }

        StringToLower(&input_type);
        if (input_type != "photometric" && input_type != "geometric") {
            std::cout << "ERROR: Invalid input type - supported values are "
                        "'photometric' and 'geometric'."
                    << std::endl;
            return EXIT_FAILURE;
        }

        mvs::StereoFusion fuser(*options.stereo_fusion, dense_workspace_path,
                                workspace_format, pmvs_option_name, input_type);

        fuser.Start();
        fuser.Wait();

        std::cout << "Writing output: " << dense_output_path << std::endl;
        WriteBinaryPlyPoints(dense_output_path, fuser.GetFusedPoints());
        mvs::WritePointsVisibility(dense_output_path + ".vis",
                                    fuser.GetFusedPointsVisibility());

        return EXIT_SUCCESS;
    }

    int PoissonMesher(const std::string& project_path) {
        std::string poisson_input_path;
        std::string poisson_output_path;

        OptionManager options;
        options.AddRequiredOption("poisson_input_path", &poisson_input_path);
        options.AddRequiredOption("possion_output_path", &poisson_output_path);
        options.AddPoissonMeshingOptions();
        options.Read(project_path);

        CHECK(mvs::PoissonMeshing(*options.poisson_meshing, poisson_input_path, poisson_output_path));

        return EXIT_SUCCESS;
    }

    int DelaunayMesher(const std::string& project_path) {
        #ifndef CGAL_ENABLED
        std::cerr << "ERROR: Delaunay meshing requires CGAL, which is not "
                    "available on your system."
                    << std::endl;
        return EXIT_FAILURE;
        #else   // CGAL_ENABLED
        std::string delaunay_input_path;
        std::string delaunay_input_type = "dense";
        std::string delaunay_output_path;

        OptionManager options;
        options.AddRequiredOption(
            "delaunay_input_path", &delaunay_input_path,
            "Path to either the dense workspace folder or the sparse reconstruction");
        options.AddDefaultOption("delaunay_input_type", &delaunay_input_type, "{dense, sparse}");
        options.AddRequiredOption("delaunay_output_path", &delaunay_output_path);
        options.AddDelaunayMeshingOptions();
        options.Read(project_path);

        StringToLower(&delaunay_input_type);
        if (delaunay_input_type == "sparse") {
            mvs::SparseDelaunayMeshing(*options.delaunay_meshing, delaunay_input_path,
                                    delaunay_output_path);
        } else if (delaunay_input_type == "dense") {
            mvs::DenseDelaunayMeshing(*options.delaunay_meshing, delaunay_input_path,
                                    delaunay_output_path);
        } else {
            std::cout << "ERROR: Invalid input type - "
                        "supported values are 'sparse' and 'dense'."
                    << std::endl;
            return EXIT_FAILURE;
        }

        return EXIT_SUCCESS;
        #endif  // CGAL_ENABLED
    }
}
