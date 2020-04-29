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
}
