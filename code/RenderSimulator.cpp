#include "RenderSimulator.h"
#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"
#include "OSPRayMDL.h"
#include "Utils.h"
#define STB_IMAGE_IMPLEMENTATION   // use of stb functions once and for all
#include "stb_image.h"
#define IDX(X) ((X)-1)

//for mkdir
#if defined(_WIN32)
#include <direct.h>
#else
#include <sys/stat.h> 
#endif

/*
 * length unit: cm
 */
namespace KohyoungConstant
{
    // 2020.09.23 version
    int NUM_2DLIGHT = 3;
    int TOP = 1;
    int MIDDLE = 2;
    int BOTTOM = 3;

    /*
     * Camera / lens
     */
    float DISTANCE_CAMERA = 27;
    float PIXEL_SIZE = 15 / 1000000 * 100; // cm
    float NA_LENS = 0.009;

    /*
     * 3D Lights
     */
    float DISTANCE_3D_LIGHTS = 48.6420; // cm
    float ELEVATION_ANGLE_3D_LIGHTS = 65; // degree
    float DIFFUSE_ANGLE_3D_LIGHTS = 12; // degree
    float WAVE_LENGTH_EVEN = 0.1008; // cm
    float WAVE_LENGTH_ODD = 0.0819; // cm
    /*
     * 2D Lights
     */
    // TOP
    float DIFFUSER_DISTANCE_TOP = 25.45;
    float DIFFUSER_INNER_DIAMETER = 10.8;
    float DIFFUSER_OUTER_DIAMETER = 13.5;
    float DIFFUSER_RING_DIAMETER = (DIFFUSER_INNER_DIAMETER + DIFFUSER_OUTER_DIAMETER) / 2.0f;
    float DIFFUSER_RING_RADIUS = DIFFUSER_RING_DIAMETER / 2.0f;
    float DIFFUSER_RING_WIDTH = (DIFFUSER_OUTER_DIAMETER - DIFFUSER_INNER_DIAMETER) / 2.0f;
    float DIFFUSER_HEIGHT_TOP = sqrt(DIFFUSER_DISTANCE_TOP * DIFFUSER_DISTANCE_TOP - DIFFUSER_RING_RADIUS * DIFFUSER_RING_RADIUS);

    // MIDDLE
    float DIFFUSER_FRAME_DIAMETER_MIDDLE = 13.3 * cos(52 * piOver180) * 2;
    float DIFFUSER_HEIGHT_MIDDLE = 13.3 * sin(52 * piOver180);

    // BOTTOM
    float DIFFUSER_FRAME_DIAMETER_BOTTOM = 11.18 * cos(40 * piOver180) * 2;
    float DIFFUSER_HEIGHT_BOTTOM = 11.18 * sin(40 * piOver180);

    // middle, bottom share
    float DIFFUSER_QUAD_HEIGHT = 0.8f;
    float DIFFUSER_QUAD_WIDTH = 3.6f;
};

Vec3f cross(const Vec3f& v1, const Vec3f& v2)
{
    return Vec3f(v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x);
}

bool SampleRenderer::Init(int argc, char** argv)
{
    this->progressiveRendering = false;
    bool printHelp = false;

    bool swapXY = false;
    bool swapYZ = false;
    bool useObj = false;
    bool useFOV = false;
    bool useShift = false;

    std::string currentMaterial = "::OBJMaterial";

    std::string planeMaterial;
    std::string sphereMaterial;
    // std::string targetFilePath;

    // Tone Mapping
    this->toneMapping = false;

    // Parse args
    for (int i = 1; i < argc; ++i)
    {
        std::string arg(argv[i]);
        std::string argNext(i < argc - 1 ? argv[i + 1] : "");
        if (i==1)
        {
            targetFilePath = arg;
        }

        if (arg == "-h" || arg == "--help")
        {
            printHelp = true;
        }
        else if (arg == "--swapXY")
        {
            swapXY = true;
        }
        else if (arg == "--swapYZ")
        {
            swapYZ = true;
        }
        else if (arg == "--useObj")
        {
            useObj = true;
        }
        else if (arg == "--useFOV")
        {
            useFOV = true;
        }
        else if (arg == "--useShift")
        {
            useShift = true;
        }
        else if (arg == "--alignCameraCenter")
        {
            alignCameraCenter = true;
        }
        else if (arg == "--exportImage3D")
        {
            this->exportImage3D = true;
            this->progressiveRendering = true;
        }
        else if (arg == "--exportImage2D")
        {
            this->exportImage2D = true;
            this->progressiveRendering = true;
        }
        else if (arg == "--removeInterreflection")
        {
            this->exportImageRemoveInterreflection = true;
        }
        else if (arg == "--samplesPerPixel")
        {
            this->samplesPerPixel = atoi(argv[++i]);
            //this->exportImageFrameNumber = atoi(argv[++i]);
        }
        else if (arg == "--useTonemapping")
        {
            this->toneMapping = atoi(argv[++i]);
        }
        // else if (arg == "--spotLightIntensity"){
        //     this->spotLightIntensity = atof(argv[++i]);
        // }
        else if (arg == "--screenHeight"){
            this->screenHeight = atof(argv[++i]);
            std::cout << "Screen height set to " << this->screenHeight << std::endl;
        }
        
    }
    if(this->exportImage3D) std::cout << "Export Image 3D" << std::endl;
    if(this->exportImage2D) std::cout << "Export Image 2D" << std::endl;

    // Print help
    if (printHelp)
    {
        std::cout << "VisRTX mesh loader" << std::endl;
        std::cout << "Usage: visRtxSampleMeshViewer [params]" << std::endl;
        std::cout << "Options:" << std::endl;
        std::cout << " ::$material : Use MDL material with given name for the following meshes (Available: ::OBJMaterial, ::Alloy, ::Metal, ::MetallicPaint, ::CarPaint, ::Principled, ::ThinGlass, ::Glass)" << std::endl;
        std::cout << " $path.obj : Any number of OBJ meshes" << std::endl;
        std::cout << " --plane : Add a ground plane with the current material" << std::endl;
        std::cout << " --sphere : Add a sphere with the current material" << std::endl;
        std::cout << " --swapXY : Swap X/Y axes" << std::endl;
        std::cout << " --swapYZ : Swap Y/Z axes" << std::endl;
        std::cout << " --useObj : use obj file format" << std::endl;
        
        return false;
    }

    // ---------------------------------------------------------------------------------------------------------------

    // Ambient
    this->ambientColor.r = 0.0f;
    this->ambientColor.g = 0.0f;
    this->ambientColor.b = 0.0f;

    // Camera
    this->rotationHorizontal = 270.0f;
    this->rotationVertical = 0.0f;
    this->distance = KohyoungConstant::DISTANCE_CAMERA;
    
    // DOF
    this->depthOfField = true;
    this->focalDistance = distance; // Denotes the world-space distance from the camera's aperture to the focal plane.
    this->apertureRadius = KohyoungConstant::NA_LENS * focalDistance; // Denotes the radius of the camera's aperture in scene units.

    // Renderer
    this->numBouncesMin = 1;
    this->numBouncesMax = 8;

    VisRTX::Context* context = VisRTX_GetContext();

    std::cout << "Loading resource : " << targetFilePath.c_str() << std::endl;
    
    // std::set<int> mat_types_set;
    // std::vector<float> materialmap_data_array;
    // for(int i = 0; i < 3 * width * height; i += 3)
    // {
    //     // flip the order of every 3 bytes
    //     unsigned char tmp = materialmap[i];
    //     materialmap_data_array.push_back((float)tmp);
    //     if(i == 70){
    //         std::cout << (int)tmp << std::endl;
    //         std::cout << (float)tmp << std::endl;
    //     }
    //     mat_types_set.insert((int)tmp);
    // }
    // for(int m : mat_types_set){
    //     std::cout << m << std::endl;
    // }
    
    
    // Create geometries
    /*for (Mesh& mesh : meshes)
    {

    }*/

    this->useObj = useObj;
    this->useShift = useFOV ? false : useShift; // shift only when using component image
    if(useObj){this->AddGeometryUsingObj(targetFilePath, context);}
    else{this->AddGeometryUsingHeightMap(targetFilePath, useFOV, context);}

    this->AddLight(context);
    return true;
}

void SampleRenderer::AddGeometryUsingObj(std::string targetFilePath, VisRTX::Context* context)
{   
    struct Mesh
    {
        std::string path;
        std::string material;

        std::vector<VisRTX::Vec3f> vertices;
        std::vector<VisRTX::Vec3f> normals;
        std::vector<VisRTX::Vec2f> texcoords;

        std::vector<VisRTX::Material*> material_types;  // material types are UNIQUE!!
        std::vector<VisRTX::Material*> materials;       // material for each faces

        std::vector<VisRTX::Vec3ui> triangles;
        VisRTX::MDLMaterial* mdl;

        VisRTX::Vec3f shiftOffset; // Offset when using useShift
    };

    Mesh mesh;       // use only single mesh!.
    std::string base_dir = "../data/sample_data_obj";
    mesh.path = base_dir + "/" + targetFilePath;
    
    Timer timer;

    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    std::string warn;
    std::string err;
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, 
        (mesh.path+std::string(".obj")).c_str(), base_dir.c_str() ) ;

    if (!err.empty())
    {
        std::cerr << err << std::endl;
        return;
    } else {
        std::cout << targetFilePath << " mesh loaded" << std::endl;
    }

    // Load general info from ini file. ("general_info.ini") & save scaleFactors
    std::string line;
    std::string targetGeneralINIFile = "../data/sample_data/" + targetFilePath + "/" + "general_info.ini";
    std::map<std::string, std::map<std::string, float>> general_ini_sections;
    std::ifstream general_ini_file;
    general_ini_file.open(targetGeneralINIFile.c_str());
    std::map<std::string, float> general_current_section = std::map<std::string, float>();
    std::string curr_section_name;
    while (std::getline (general_ini_file, line)) {
        if(line.rfind("[",0) == 0){
            if(!general_current_section.empty()){
                general_ini_sections[curr_section_name] = general_current_section;
            }
            general_current_section = std::map<std::string, float>();
            curr_section_name = (line[line.size() - 1] == '\r') ? line.substr(1, line.size() - 3) : line.substr(1, line.size() - 2);
        }
        else{
            std::stringstream s (line);
            std::string key_str, value_str;
            std::getline (s, key_str, '=');
            std::getline (s, value_str, '=');
            // remove carriage return
            if (!value_str.empty() && value_str[value_str.size() - 1] == '\r')
                value_str.erase(value_str.size() - 1);
            general_current_section[key_str] = std::stof(value_str);
        }
    }
    if(!general_current_section.empty()){
        general_ini_sections[curr_section_name] = general_current_section;
    }
    general_ini_file.close();
    Vec3f scales[1] =
            {
                    Vec3f(general_ini_sections["Camera"]["ScaleFactorX"] / 1000000.0 * 100.0,
                          1.0 / 1000000.0 * 100.0,
                          general_ini_sections["Camera"]["ScaleFactorY"] / 1000000.0 * 100.0)
            };

    // translate the component
    if(this->useShift)
    {
        // Load width & height of component image
        int width, height, nrChannels;
        std::string targetLabelMapFile = "../data/sample_data/" + targetFilePath + "/" + "comp" + "_label_map.bmp";
        unsigned char *materialmap = stbi_load(targetLabelMapFile.c_str(), &width, &height, &nrChannels, 0);
        mesh.shiftOffset = VisRTX::Vec3f(
                (general_ini_sections["Image"]["StitchingLeftTopX"] + width / 2.0 - general_ini_sections["Camera"]["ResolutionX"] / 2.0) * scales[0].x,
                0,
                (general_ini_sections["Image"]["StitchingLeftTopY"] + height / 2.0 - general_ini_sections["Camera"]["ResolutionY"] / 2.0) * scales[0].z
                );
    }
    else mesh.shiftOffset = VisRTX::Vec3f(0.f, 0.f, 0.f);

    // Load material info from ini file. ("material_info.ini")
    std::string targetMaterialINIFile = "../data/sample_data/" + targetFilePath + "/" + "material_info.ini";
    std::vector<std::map<std::string, std::string>> ini_sections;
    std::ifstream ini_file;
    ini_file.open(targetMaterialINIFile.c_str());
    std::map<std::string, std::string> current_section = std::map<std::string, std::string>();
    while (std::getline (ini_file, line)) {
        if(line.rfind("[",0) == 0){
            if(!current_section.empty()){
                ini_sections.push_back(current_section);
            }
            current_section = std::map<std::string, std::string>();
        }
        else{
            std::stringstream s (line);
            std::string key_str, value_str;
            std::getline (s, key_str, '=');
            std::getline (s, value_str, '=');
            // remove carriage return
            if (!value_str.empty() && value_str[value_str.size() - 1] == '\r')
                value_str.erase(value_str.size() - 1);
            current_section[key_str] = value_str;
        }
    }
    ini_sections.push_back(current_section);
    ini_file.close();

    const std::string osprayMDLSource((const char*)OSPRay_mdl, sizeof(OSPRay_mdl));

    for(size_t m = 0; m < materials.size(); m++){
        std::cout << "Material ID " << m << " : " << materials[m].name << std::endl;
        std::string target_material_type;
        if(
            materials[m].name.compare("tin") == 0 || 
            materials[m].name.compare("steel") == 0 ||
            materials[m].name.compare("copper") == 0 ||
            materials[m].name.compare("lead") == 0 ||
            materials[m].name.compare("solder_paste") == 0 ||
            materials[m].name.compare("pad") == 0
        ){
            target_material_type = "::Alloy";
        } else {
            target_material_type = "::Principled";
        }

        MDLMaterial* mdl = this->LoadMDL("::ospray" + target_material_type, osprayMDLSource, {}, 
            compilationType, 0, materials[m].name);

        // assign material params
        std::map<std::string, std::string> section;
        for(auto sec: ini_sections)
        {
            if(sec[std::string("material_name")].compare(materials[m].name) == 0)
                section = sec;
        }
        uint32_t numParameters = mdl->GetParameterCount();

        for (uint32_t i = 0; i < numParameters; ++i)
        {
            const char* name = mdl->GetParameterName(i);
            VisRTX::ParameterType pt = mdl->GetParameterType(name);

            MaterialGUI& mGUI = this->materials.back();

            std::map<std::string, std::string>::iterator it;
            it = section.find(std::string(name));
            if (pt == VisRTX::ParameterType::FLOAT)
            {
                if(it != section.end())
                {
                    mdl->SetParameterFloat(name, std::stof(it->second));
                }
            }
            else if (pt == VisRTX::ParameterType::DOUBLE)
            {
                if(it != section.end())
                {
                    mdl->SetParameterDouble(name, std::stof(it->second));
                }
            }
            else if (pt == VisRTX::ParameterType::INT)
            {
                if(it != section.end())
                {
                    mdl->SetParameterInt(name, std::stoi(it->second));
                }
            }
            else if (pt == VisRTX::ParameterType::COLOR)
            {
                if(target_material_type.compare("::Alloy") == 0)
                {
                    if(std::string(name).compare("Color") == 0)
                    {
                        if(section.find(std::string("Color_r")) != section.end() &&
                           section.find(std::string("Color_g")) != section.end() &&
                           section.find(std::string("Color_b")) != section.end())
                        {

                            mdl->SetParameterColor(name, Vec3f(std::stof(section["Color_r"]),
                                                               std::stof(section["Color_g"]),
                                                               std::stof(section["Color_b"]))
                            );
                        }
                    }
                    else if(std::string(name).compare("edgeColor") == 0)
                    {
                        if(section.find(std::string("edgeColor_r")) != section.end() &&
                           section.find(std::string("edgeColor_g")) != section.end() &&
                           section.find(std::string("edgeColor_b")) != section.end())
                        {

                            mdl->SetParameterColor(name, Vec3f(std::stof(section["edgeColor_r"]),
                                                               std::stof(section["edgeColor_g"]),
                                                               std::stof(section["edgeColor_b"]))
                            );
                        }
                    }
                }
                else if(target_material_type.compare("::Principled") == 0)
                {
                    if(std::string(name).compare("baseColor") == 0)
                    {
                        if(section.find(std::string("baseColor_r")) != section.end() &&
                           section.find(std::string("baseColor_g")) != section.end() &&
                           section.find(std::string("baseColor_b")) != section.end())
                        {
                            mdl->SetParameterColor(name, Vec3f(std::stof(section["baseColor_r"]),
                                                               std::stof(section["baseColor_g"]),
                                                               std::stof(section["baseColor_b"]))
                            );
                        }
                    }
                    else if(std::string(name).compare("roughness") == 0)
                    {
                        if(section.find(std::string("roughness")) != section.end())
                        {
                            mdl->SetParameterFloat(name, std::stof(section["roughness"]));
                        }
                    }
                }
            }
            mGUI.LoadCurrentValues();
        }

        mdl->Compile();


        if(
            materials[m].name.compare("green_plastic") == 0 ||
            materials[m].name.compare("board") == 0
        )
        {
            this->planeMat = mdl;
        }
        mesh.material_types.push_back(mdl);
    }

    //////////////////////////////////////
    // New: Remove duplicated vertices
    //////////////////////////////////////

    typedef std::tuple <int,int,int> vertex_pos_nor_tex_tuple;
    std::map<vertex_pos_nor_tex_tuple, int> vertex_id_map;
    std::vector<int> face_indice_temp;

    for (size_t s = 0; s < shapes.size(); s++)
    {
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++)
        {
            int fv = shapes[s].mesh.num_face_vertices[f];

            if (fv == 3)
            {
                uint32_t vertexOffset = (uint32_t)mesh.vertices.size();
                face_indice_temp.clear();
                for (size_t v = 0; v < fv; v++)
                {
                    // access to vertex
                    tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];

                    int vertex_index = 0;
                    int normal_index = 0;
                    int texcoord_index = 0;
                    if (idx.vertex_index >= 0){vertex_index = idx.vertex_index;}
                    if (idx.normal_index >= 0){normal_index = idx.normal_index;}
                    if (idx.texcoord_index >= 0){texcoord_index = idx.texcoord_index;}

                    vertex_pos_nor_tex_tuple vertex_key = std::make_tuple(idx.vertex_index, idx.normal_index, idx.texcoord_index);

                    // add new vertex if it is newly seen.
                    if(vertex_id_map.find(vertex_key) == vertex_id_map.end()){
                        vertex_id_map[vertex_key] = vertex_id_map.size();

                        if (idx.vertex_index >= 0)
                        {
                            tinyobj::real_t vx = attrib.vertices[3 * idx.vertex_index + 0];
                            tinyobj::real_t vy = attrib.vertices[3 * idx.vertex_index + 1];
                            tinyobj::real_t vz = attrib.vertices[3 * idx.vertex_index + 2];

                            mesh.vertices.push_back(VisRTX::Vec3f(vx + mesh.shiftOffset.x,
                                                                  vy + mesh.shiftOffset.y,
                                                                  vz + mesh.shiftOffset.z));
                        }

                        if (idx.normal_index >= 0)
                        {
                            tinyobj::real_t nx = attrib.normals[3 * idx.normal_index + 0];
                            tinyobj::real_t ny = attrib.normals[3 * idx.normal_index + 1];
                            tinyobj::real_t nz = attrib.normals[3 * idx.normal_index + 2];

                            if(useShift)
                            mesh.normals.push_back(VisRTX::Vec3f(nx, ny, nz));
                        }

                        if (idx.texcoord_index >= 0)
                        {
                            tinyobj::real_t tx = attrib.texcoords[2 * idx.texcoord_index + 0];
                            tinyobj::real_t ty = attrib.texcoords[2 * idx.texcoord_index + 1];

                            mesh.texcoords.push_back(VisRTX::Vec2f(tx, ty));
                        }
                    }
                    int reordered_vertex_id = vertex_id_map[vertex_key];

                    face_indice_temp.push_back(reordered_vertex_id);
                }
                index_offset += fv;

                // add triangle (vertex indices)
                mesh.triangles.push_back(VisRTX::Vec3ui(face_indice_temp[0], face_indice_temp[1], face_indice_temp[2]));
                int face_material_id = shapes[s].mesh.material_ids[f];
                mesh.materials.push_back(mesh.material_types[face_material_id]);
            }
        }
    }
    std::cout << "Vertex Size: " << mesh.vertices.size() << std::endl;
    std::cout << "Load OBJ: " << mesh.path << ": " << timer.GetElapsedMilliseconds() << " ms" << std::endl;

    // Create Geometry
    VisRTX::TriangleGeometry* _triangleGeo = context->CreateTriangleGeometry((uint32_t)mesh.triangles.size(), mesh.triangles.data(), (uint32_t)mesh.vertices.size(), mesh.vertices.data(), mesh.normals.empty() ? nullptr : mesh.normals.data());
    _triangleGeo->SetTexCoords(mesh.texcoords.empty() ? nullptr : mesh.texcoords.data());
    _triangleGeo->SetMaterials(mesh.materials.data());
    for (VisRTX::Material* mat : mesh.material_types)
        mat->Release();

    this->triangleGeo = _triangleGeo;
    model->AddGeometry(_triangleGeo);
    this->geometryIsAddedtoModel = true;
    this->releaseLater.insert(_triangleGeo);
    this->heightmapGeo = 0;

    // Set screenHeight with scale & resolution in general info only if screen height is not set
    if(this->screenHeight < 0)
        this->screenHeight = general_ini_sections["Camera"]["ResolutionY"] * scales[0].z;

    /*
    *  Create plane with material "green_plastic" or "board"
    */
    const float planeSize = 100.0f;
    const float y = 0.f; // floor plane. y=0
    Vec3f planeVertices[4] = {
        Vec3f(-planeSize, y, planeSize),
        Vec3f(planeSize, y,  planeSize),
        Vec3f(planeSize, y,  -planeSize),
        Vec3f(-planeSize, y, -planeSize)
    };

    Vec2f planeTexCoords[4] = {
        Vec2f(0.0f, 0.0f),
        Vec2f(1.0f, 0.0f),
        Vec2f(1.0f, 1.0f),
        Vec2f(0.0f, 1.0f)
    };

    Vec3ui planeTriangles[2] = {
        Vec3ui(0, 1, 2),
        Vec3ui(0, 2, 3)
    };

    TriangleGeometry * plane = context->CreateTriangleGeometry();
    plane->SetTriangles(2, planeTriangles, 4, planeVertices);
    plane->SetTexCoords(planeTexCoords);
    plane->SetMaterial(planeMat);
    planeMat->Release();

    model->AddGeometry(plane);
    this->releaseLater.insert(plane);
}

void SampleRenderer::AddGeometryUsingHeightMap(std::string targetFilePath, bool useFOV, VisRTX::Context* context)
{
    std::string fov_or_comp = useFOV?"fov":"comp";
    std::string targetPathFolder = "../data/sample_data/" + targetFilePath;
    std::string targetLabelMapFile = "../data/sample_data/" + targetFilePath + "/" + fov_or_comp + "_label_map.bmp";
    std::string targetHeightMapFile = "../data/sample_data/" + targetFilePath + "/" + fov_or_comp + "_height.csv";
    std::string targetMaterialINIFile = "../data/sample_data/" + targetFilePath + "/" + "material_info.ini";
    std::string targetGeneralINIFile = "../data/sample_data/" + targetFilePath + "/" + "general_info.ini";
    std::string targetNormalMapFile = "../data/common/noise.jpg";

    /* 
    * 1. Create Heightmap / Labelmap 
    */

    // 1-(1) Load labelmap ("comp_label_map.bmp")
    int width, height, nrChannels;
    unsigned char *materialmap = stbi_load(targetLabelMapFile.c_str(), &width, &height, &nrChannels, 0);
    std::cout << "Width: " << width << " Height: " << height << " Channels : " << nrChannels << std::endl;

    // 1-(2) Load heightmap from csv file ("comp_height.csv")
    std::ifstream f;
    f.open(targetHeightMapFile.c_str());
    std::string line, val;                  
    std::vector<float> heightmap_data_array;    
    std::vector<float> plane_heightmap_data_array;    
    float heightmap_maxheight = 0;
    float heightmap_minheight = 1000;

    int heightmapWidth = 0; 
    int heightmapHeight = 0;
    while (std::getline (f, line)) {        /* read each line */
        heightmapHeight++;
        heightmapWidth = 0;
        std::stringstream s (line);         /* stringstream line */
        while (getline (s, val, ',')){
            float height = std::stof(val);
            heightmap_data_array.push_back(height);
            plane_heightmap_data_array.push_back(0);
            if(height>heightmap_maxheight){heightmap_maxheight = height;}
            if(height<heightmap_minheight){heightmap_minheight = height;}
            heightmapWidth++;
        }
    }
    f.close();
    std::cout << "HeightWidth: " << heightmapWidth << " HeightHeight: " << heightmapHeight << std::endl;
    std::cout << "Max height: " << heightmap_maxheight << ", Min height: " << heightmap_minheight << std::endl;

    // 1-(3) Create VisRTX Textures
    VisRTX::Texture* heightmapTexture = context->CreateTexture(VisRTX::Vec2ui(width, height), 
        VisRTX::TextureFormat::R32F, heightmap_data_array.data());
    heightmapTexture->SetWrapMode(TextureWrapMode::CLAMP_TO_EDGE, TextureWrapMode::CLAMP_TO_EDGE);
    heightmapTexture->SetMipLevel((unsigned int)(this->maxMipLevel));
    heightmapTexture->SetFiltering(TextureFiltering::LINEAR, TextureFiltering::LINEAR);
    int heightmapTextureId = heightmapTexture->GetTextureSamplerId();
    std::cout << "Heightmap texture ID: " << heightmapTextureId << std::endl;

    VisRTX::Texture* materialmapTexture = context->CreateTexture(VisRTX::Vec2ui(width, height), 
        VisRTX::TextureFormat::RGB8, materialmap);
    materialmapTexture->SetWrapMode(TextureWrapMode::CLAMP_TO_EDGE, TextureWrapMode::CLAMP_TO_EDGE);
    materialmapTexture->SetMipLevel((unsigned int)(this->maxMipLevel));
    materialmapTexture->SetFiltering(TextureFiltering::LINEAR, TextureFiltering::LINEAR);
    int materialmapTextureId = materialmapTexture->GetTextureSamplerId();
    std::cout << "Materialmap texture ID: " << materialmapTextureId << std::endl;

    // 1-(4) Load noise map ("noise.jpg")
    int noiseWidth, noiseHeight, noisenrChannels;
    unsigned char *noiseNormalMap = stbi_load(targetNormalMapFile.c_str(), &noiseWidth, &noiseHeight, &noisenrChannels, 0);
    std::cout << "Noise normal map Width: " << noiseWidth << " Height: " << noiseHeight << " Channels : " << noisenrChannels << std::endl;
    
    VisRTX::Texture* noiseNormalMapTexture = context->CreateTexture(VisRTX::Vec2ui(noiseWidth, noiseHeight), 
        VisRTX::TextureFormat::RGB8, noiseNormalMap);
    noiseNormalMapTexture->SetWrapMode(TextureWrapMode::REPEAT, TextureWrapMode::REPEAT);
    noiseNormalMapTexture->SetFiltering(TextureFiltering::LINEAR, TextureFiltering::LINEAR);
    int noiseNormalmapTextureId = noiseNormalMapTexture->GetTextureSamplerId();
    std::cout << "Noise normal map texture ID: " << noiseNormalmapTextureId << std::endl;



    /* 
    * 2. Create materials
    */
    // 2-(1) Load material info from ini file. ("material_info.ini")
    std::vector<std::map<std::string, std::string>> ini_sections;
    std::ifstream ini_file;
    ini_file.open(targetMaterialINIFile.c_str());
    std::map<std::string, std::string> current_section = std::map<std::string, std::string>();
    while (std::getline (ini_file, line)) {        
        if(line.rfind("[",0) == 0){
            if(!current_section.empty()){
                ini_sections.push_back(current_section);
            }
            current_section = std::map<std::string, std::string>();
        }
        else{
            std::stringstream s (line);
            std::string key_str, value_str;
            std::getline (s, key_str, '=');
            std::getline (s, value_str, '=');
            // remove carriage return
            if (!value_str.empty() && value_str[value_str.size() - 1] == '\r')
                value_str.erase(value_str.size() - 1);

            current_section[key_str] = value_str;
        }
    }
    ini_sections.push_back(current_section);
    ini_file.close();

    // 2-(2) Create MDL materials.
    // Load OSPRay materials from memory
    const std::string osprayMDLSource((const char*)OSPRay_mdl, sizeof(OSPRay_mdl));

    std::vector<VisRTX::Material*> material_types;
    std::vector<unsigned int> noiseNormalMapIds;
    
    for(auto & section : ini_sections){
        std::string material_name = std::string(section["material_name"]);
        std::cout << "Material name : " << material_name << std::endl;
        
        std::string target_material_type;
        if(
            material_name.compare("tin") == 0 || 
            material_name.compare("steel") == 0 ||
            material_name.compare("copper") == 0 ||
            material_name.compare("lead") == 0 ||
            material_name.compare("solder_paste") == 0 ||
            material_name.compare("pad") == 0
        ){
            target_material_type = "::Alloy";
        } else {
            target_material_type = "::Principled";
        }
        MDLMaterial* mdl = this->LoadMDL("::ospray" + target_material_type, osprayMDLSource, {}, 
                compilationType, 0, material_name);


        // assign material params
        uint32_t numParameters = mdl->GetParameterCount();

        for (uint32_t i = 0; i < numParameters; ++i)
        {
            const char* name = mdl->GetParameterName(i);
            VisRTX::ParameterType pt = mdl->GetParameterType(name);

            MaterialGUI& mGUI = this->materials.back();

            std::map<std::string, std::string>::iterator it;
            it = section.find(std::string(name));

            if (pt == VisRTX::ParameterType::FLOAT)
            {
                if(it != section.end())
                {
                    mdl->SetParameterFloat(name, std::stof(it->second));
                }
            }
            else if (pt == VisRTX::ParameterType::DOUBLE)
            {
                if(it != section.end())
                {
                    mdl->SetParameterDouble(name, std::stof(it->second));
                }
            }
            else if (pt == VisRTX::ParameterType::INT)
            {
                if(it != section.end())
                {
                    mdl->SetParameterInt(name, std::stoi(it->second));
                }
            }
            else if (pt == VisRTX::ParameterType::COLOR)
            {
                if(target_material_type.compare("::Alloy") == 0)
                {
                    if(std::string(name).compare("Color") == 0)
                    {
                        if(section.find(std::string("Color_r")) != section.end() &&
                           section.find(std::string("Color_g")) != section.end() &&
                           section.find(std::string("Color_b")) != section.end())
                        {
                            mdl->SetParameterColor(name, Vec3f(std::stof(section["Color_r"]),
                                                               std::stof(section["Color_g"]),
                                                               std::stof(section["Color_b"]))
                            );
                        }
                    }
                    else if(std::string(name).compare("edgeColor") == 0)
                    {
                        if(section.find(std::string("edgeColor_r")) != section.end() &&
                           section.find(std::string("edgeColor_g")) != section.end() &&
                           section.find(std::string("edgeColor_b")) != section.end())
                        {
                            mdl->SetParameterColor(name, Vec3f(std::stof(section["edgeColor_r"]),
                                                               std::stof(section["edgeColor_g"]),
                                                               std::stof(section["edgeColor_b"]))
                            );
                        }
                    }
                }
                else if(target_material_type.compare("::Principled") == 0)
                {
                    if(std::string(name).compare("baseColor") == 0)
                    {
                        if(section.find(std::string("baseColor_r")) != section.end() &&
                           section.find(std::string("baseColor_g")) != section.end() &&
                           section.find(std::string("baseColor_b")) != section.end())
                        {
                            mdl->SetParameterColor(name, Vec3f(std::stof(section["baseColor_r"]),
                                                               std::stof(section["baseColor_g"]),
                                                               std::stof(section["baseColor_b"]))
                            );
                        }
                    }
                }
            }
            mGUI.LoadCurrentValues();
        }

        mdl->Compile();


        if(target_material_type == "::Alloy"){
            noiseNormalMapIds.push_back(noiseNormalmapTextureId);
            std::cout << noiseNormalmapTextureId << " noise normal" << std::endl;
        } else {
            noiseNormalMapIds.push_back(0);
        }

        if(
            material_name.compare("green_plastic") == 0 ||
            material_name.compare("board") == 0
        )
        {
            this->planeMat = mdl;
        }

        material_types.push_back(mdl);
    }

    /* 
    * 3. Create heightmap geometry object.
    */

    // 3-(1) Load general info from ini file. ("general_info.ini") & save the scaleFactors
    std::map<std::string, std::map<std::string, float>> general_ini_sections;
    std::ifstream general_ini_file;
    general_ini_file.open(targetGeneralINIFile.c_str());
    std::map<std::string, float> general_current_section = std::map<std::string, float>();
    std::string curr_section_name;
    while (std::getline (general_ini_file, line)) {
        if(line.rfind("[",0) == 0){
            if(!general_current_section.empty()){
                general_ini_sections[curr_section_name] = general_current_section;
            }
            general_current_section = std::map<std::string, float>();
            curr_section_name = (line[line.size() - 1] == '\r') ? line.substr(1, line.size() - 3) : line.substr(1, line.size() - 2);
        }
        else{
            std::stringstream s (line);
            std::string key_str, value_str;
            std::getline (s, key_str, '=');
            std::getline (s, value_str, '=');
            // remove carriage return
            if (!value_str.empty() && value_str[value_str.size() - 1] == '\r')
                value_str.erase(value_str.size() - 1);
            general_current_section[key_str] = std::stof(value_str);
        }
    }
    if(!general_current_section.empty()){
        general_ini_sections[curr_section_name] = general_current_section;
    }
    general_ini_file.close();
    Vec3f scales[1] =
    {
        Vec3f(general_ini_sections["Camera"]["ScaleFactorX"] / 1000000.0 * 100.0,
              1.0 / 1000000.0 * 100.0,
              general_ini_sections["Camera"]["ScaleFactorY"] / 1000000.0 * 100.0)
    };

    // 3-(2) Create heightmap geometry
    VisRTX::HeightMapGeometry* _heightMapGeo = context->CreateHeightMapGeometry(1, scales);
    if(this->useShift)
        _heightMapGeo->SetCenterPosition(
            (general_ini_sections["Image"]["StitchingLeftTopX"] + width / 2.0 - general_ini_sections["Camera"]["ResolutionX"] / 2.0) * scales[0].x ,
            (general_ini_sections["Image"]["StitchingLeftTopY"] + height / 2.0 - general_ini_sections["Camera"]["ResolutionY"] / 2.0) * scales[0].z
            );
    else _heightMapGeo->SetCenterPosition(0.f, 0.f);
    _heightMapGeo->SetMaterials(material_types.size(), material_types.data());
    _heightMapGeo->SetHeightMapSamplerIds(heightmapTextureId, materialmapTextureId);
    _heightMapGeo->SetNoiseNormalMapSamplerIds(noiseNormalMapIds.size(), noiseNormalMapIds.data());

    _heightMapGeo->SetHeightmapSize(Vec2f((float)width, (float)height), heightmap_maxheight, heightmap_minheight);
    _heightMapGeo->SetLodLevel(this->currentLod);
    _heightMapGeo->SetRayMarchStep(this->rayMarchUnitStep, this->rayMarchMaxStepNumber);
    
    std::cout << "Heightmap geometry generated" << std::endl;
    this->heightmapGeo = _heightMapGeo;
    model->AddGeometry(_heightMapGeo);
    this->geometryIsAddedtoModel = true;
    this->triangleGeo = 0;

    // Set screenHeight with scale & resolution in general info only if screen height is not set
    if(this->screenHeight < 0)
        this->screenHeight = general_ini_sections["Camera"]["ResolutionY"] * scales[0].z;


    // 3-(3)
    if(this->useShift && this->alignCameraCenter){
        float imgHeight = general_ini_sections["Camera"]["ResolutionY"];
        float imgWidth = imgHeight * this->width / (float)this->height;
        float dx = (imgWidth - general_ini_sections["Camera"]["ResolutionX"]) * 0.5;
        float beginX = (general_ini_sections["Image"]["StitchingLeftTopX"] + dx) / imgWidth;
        float beginY = general_ini_sections["Image"]["StitchingLeftTopY"] / imgHeight;
        float sizeX = width / imgWidth;
        float sizeY = height / imgHeight;
        
        this->imageEnd = Vec2f(beginX+sizeX, 1 - beginY);
	    this->imageBegin = Vec2f(beginX, 1 - (beginY+sizeY));
    }

    /*
    * 4. Create plane with material "green_plastic" or "board"
    */
    const float planeSize = 100.0f;
    const float y = heightmap_minheight * 1.0 / 1000000.0 * 100.0; // floor plane. minimum height of the heightmap (cm)
    Vec3f planeVertices[4] = {
        Vec3f(-planeSize, y, planeSize),
        Vec3f(planeSize, y,  planeSize),
        Vec3f(planeSize, y,  -planeSize),
        Vec3f(-planeSize, y, -planeSize)
    };

    Vec2f planeTexCoords[4] = {
        Vec2f(0.0f, 0.0f),
        Vec2f(1.0f, 0.0f),
        Vec2f(1.0f, 1.0f),
        Vec2f(0.0f, 1.0f)
    };

    Vec3ui planeTriangles[2] = {
        Vec3ui(0, 1, 2),
        Vec3ui(0, 2, 3)
    };

    TriangleGeometry * plane = context->CreateTriangleGeometry();
    plane->SetTriangles(2, planeTriangles, 4, planeVertices);
    plane->SetTexCoords(planeTexCoords);
    plane->SetMaterial(planeMat);
    planeMat->Release();

    model->AddGeometry(plane);
    this->releaseLater.insert(plane);
}

void SampleRenderer::AddLight(VisRTX::Context* context)
{
    /*
    * 3D Structural Lights
    */
    this->Light3D = new LightSimulator_3D(context);
    Light3D->DISTANCE_3D_LIGHTS = KohyoungConstant::DISTANCE_3D_LIGHTS;
    Light3D->ELEVATION_ANGLE_3D_LIGHTS = KohyoungConstant::ELEVATION_ANGLE_3D_LIGHTS;
    Light3D->DIFFUSE_ANGLE_3D_LIGHTS = KohyoungConstant::DIFFUSE_ANGLE_3D_LIGHTS;
    Light3D->WAVE_LENGTH_EVEN = KohyoungConstant::WAVE_LENGTH_EVEN;
    Light3D->WAVE_LENGTH_ODD = KohyoungConstant::WAVE_LENGTH_ODD;
    Light3D->createLights(context);
    for(auto light: Light3D->stripeLights){
        renderer->AddLight(light->spotLight);
        light->Release();
    }
    /*
    * Updated
    * Ring Light
    */

    // Diffuser TOP
    this->diffuserTop = new LightSimulator_DiffuserRing(context);
    diffuserTop->height = KohyoungConstant::DIFFUSER_HEIGHT_TOP;
    diffuserTop->ringRadius = KohyoungConstant::DIFFUSER_RING_RADIUS;
    diffuserTop->ringWidth = KohyoungConstant::DIFFUSER_RING_WIDTH;
    diffuserTop->lightColor = Vec3f(1,1,1);
    diffuserTop->intensity = 55.0f;
    diffuserTop->setLightDisable();
    diffuserTop->createLights(context);
    renderer->AddLight(diffuserTop->ringLight);
    diffuserTop->ringLight->Release();


    // Diffuser MIDDLE
    this->diffuserMiddle = new LightSimulator_DiffuserQuadRing(context);
    diffuserMiddle->number = 12;
    diffuserMiddle->quadRingRadius = KohyoungConstant::DIFFUSER_FRAME_DIAMETER_MIDDLE * 0.5f;
    diffuserMiddle->quadWidth = KohyoungConstant::DIFFUSER_QUAD_WIDTH;
    diffuserMiddle->quadHeight = KohyoungConstant::DIFFUSER_QUAD_HEIGHT;
    diffuserMiddle->height = KohyoungConstant::DIFFUSER_HEIGHT_MIDDLE;
    diffuserMiddle->lightColor = Vec3f(1,1,1);
    diffuserMiddle->quadLightsIntensity = 26.0f;
    diffuserMiddle->createLights(context);
    for(auto light: diffuserMiddle->quadLights){
        renderer->AddLight(light);
        light->Release();
    }

    // Diffuser BOTTOM
    this->diffuserBottom = new LightSimulator_DiffuserQuadRing(context);
    diffuserBottom->number = 12;
    diffuserBottom->quadRingRadius = KohyoungConstant::DIFFUSER_FRAME_DIAMETER_BOTTOM * 0.5f;
    diffuserBottom->quadWidth = KohyoungConstant::DIFFUSER_QUAD_WIDTH;
    diffuserBottom->quadHeight = KohyoungConstant::DIFFUSER_QUAD_HEIGHT;
    diffuserBottom->height = KohyoungConstant::DIFFUSER_HEIGHT_BOTTOM;
    diffuserBottom->lightColor = Vec3f(1,1,1);
    diffuserBottom->quadLightsIntensity = 20.0f;
    diffuserBottom->setLightDisable();
    diffuserBottom->createLights(context);
    for(auto light: diffuserBottom->quadLights){
        renderer->AddLight(light);
        light->Release();
    }
}

void SampleRenderer::UpdateScene(bool benchmark, BenchmarkPhase benchmarkPhase, float benchmarkTimeDelta, bool pauseAllAnimations, bool& reset)
{
    UpdateLights();

    // update lod (for test)
    if(this->heightmapGeo){
        // std::cout << "Height geo update" << std::endl;
        this->heightmapGeo->SetLodLevel(this->currentLod);
        this->heightmapGeo->SetRayMarchStep(this->rayMarchUnitStep, this->rayMarchMaxStepNumber);
        this->heightmapGeo->SetNoiseNormalmap(this->useNoiseNormalMap, this->normalMapUVCoeff, this->noiseIntensity);
    }

    // render the plane only || render plane & models
    if(renderPlaneOnly)
    {
        if(useObj) model->RemoveGeometry(triangleGeo);
        else model->RemoveGeometry(heightmapGeo);
        geometryIsAddedtoModel = false;
    }
    else if(!geometryIsAddedtoModel)
    {
        if(useObj) model->AddGeometry(triangleGeo);
        else model->AddGeometry(heightmapGeo);
        geometryIsAddedtoModel = true;
    }
}

void SampleRenderer::UpdateGUI(bool& reset)
{
#ifdef VISRTX_SAMPLE_WITH_GLFW
    if (ImGui::CollapsingHeader("3D Lights"))
    {
        // disable 2D Lights
        this->diffuserTop->setLightDisable();
        this->diffuserMiddle->setLightDisable();
        this->diffuserBottom->setLightDisable();

        int current3DLightNumber = Light3D->number;

        // number of 3D stripeLights
        reset |= ImGui::SliderInt("Number##Spot", &this->Light3D->number, 1, 24);

        // default phase line angle
        reset |= ImGui::SliderFloat("Default PhaseLine Angle(deg)##Spot", &Light3D->defaultPhaseLineAngle, 0.0f, 360.0f, "%.2f");

        // make buttons for 3D Lights
        for(int iter=1; iter<=current3DLightNumber; iter++)
        {
            ImGui::RadioButton(std::to_string(iter).c_str(), &currentlyONLight3DName, iter);
            if(iter == current3DLightNumber / 2 || iter == current3DLightNumber) continue;
            else ImGui::SameLine();
        }
        for(int i=0; i<current3DLightNumber; i++)
        {
            if(i == IDX(currentlyONLight3DName))
            {
                Light3D->stripeLights[i]->setLightEnable();
            }
            else
            {
                Light3D->stripeLights[i]->setLightDisable();
            }
        }

        // RGB or Gray
        reset |= ImGui::Checkbox("GrayScale", &this->useGray);

        // Azimuth Offset
        reset |= ImGui::SliderFloat("Azimuth Offset(deg)", &this->Light3D->rotationHorizontalOffset, 0.0f, 360.0f / float(current3DLightNumber), "%.1f");
        reset |= ImGui::Checkbox("Visible##Spot", &Light3D->stripeLights[IDX(currentlyONLight3DName)]->spotLightVisible);
        reset |= ImGui::SliderFloat("Distance##Spot", &Light3D->stripeLights[IDX(currentlyONLight3DName)]->distance, 0.0f, 100.0f, "%.1f");
        reset |= ImGui::SliderFloat("rotation_Vertical##Spot", &Light3D->stripeLights[IDX(currentlyONLight3DName)]->rotationVertical, 0.0f, 180.0f, "%.1f");
        reset |= ImGui::SliderFloat("Intensity##Spot", &Light3D->stripeLights[IDX(currentlyONLight3DName)]->spotLightIntensity, 0.0f, 10000.0f, "%.2f");
        reset |= ImGui::SliderFloat("Radius##Spot", &Light3D->stripeLights[IDX(currentlyONLight3DName)]->spotLightRadius, 0.0f, 100.0f, "%.2f");
        reset |= ImGui::SliderFloat("Opening Angle##Spot", &Light3D->stripeLights[IDX(currentlyONLight3DName)]->spotLightOpeningAngle, 0.0f, 360.0f, "%.2f");
        reset |= ImGui::SliderFloat("Penumbra Angle##Spot", &Light3D->stripeLights[IDX(currentlyONLight3DName)]->spotLightPenumbraAngle, 0.0f, 360.0f, "%.2f");
        reset |= ImGui::SliderFloat("Angular Wavenumber##Spot", &Light3D->stripeLights[IDX(currentlyONLight3DName)]->angularWavenumber, 0.0f, 10000.0f, "%1f");

        if (currentlyONLight3DName % 2)
        {
            float waveNumber = Light3D->stripeLights[IDX(currentlyONLight3DName)]->angularWavenumber;
            float waveLength = 2 * pi / waveNumber;
            float rotationVertical = Light3D->stripeLights[IDX(currentlyONLight3DName)]->rotationVertical;
            float wavelengthOnBase = waveLength * Light3D->stripeLights[IDX(currentlyONLight3DName)]->distance / cos(rotationVertical * piOver180);

            ImGui::Text("WAVE_LENGTH_ODD  : %f (cm)", wavelengthOnBase);
        }
        else
        {
            float waveNumber = Light3D->stripeLights[IDX(currentlyONLight3DName)]->angularWavenumber;
            float waveLength = 2 * pi / waveNumber;
            float rotationVertical = Light3D->stripeLights[IDX(currentlyONLight3DName)]->rotationVertical;
            float wavelengthOnBase = waveLength * Light3D->stripeLights[IDX(currentlyONLight3DName)]->distance / cos(rotationVertical * piOver180);

            ImGui::Text("WAVE_LENGTH_EVEN  : %f (cm)", wavelengthOnBase);
        }

        ImGui::RadioButton("Phase_1", &Light3D->stripeLights[IDX(currentlyONLight3DName)]->currentlyONPhase, 1); ImGui::SameLine();
        ImGui::RadioButton("Phase_2", &Light3D->stripeLights[IDX(currentlyONLight3DName)]->currentlyONPhase, 2); ImGui::SameLine();
        ImGui::RadioButton("Phase_3", &Light3D->stripeLights[IDX(currentlyONLight3DName)]->currentlyONPhase, 3); ImGui::SameLine();
        ImGui::RadioButton("Phase_4", &Light3D->stripeLights[IDX(currentlyONLight3DName)]->currentlyONPhase, 4);
        //reset |= ImGui::SliderFloat("Angular Offset##Spot", &Light3D->stripeLights[IDX(currentlyONLight3DName)]->angularOffset, 0.0f, 100.0f, "%.2f");
        //reset |= ImGui::SliderFloat("PhaseLine Angle##Spot", &Light3D->stripeLights[IDX(currentlyONLight3DName)]->phaseLineAngle, -5.0f, 5.0f, "%.2f");

    }

    if (ImGui::CollapsingHeader("2D Lights"))
    {
        // disable 3D Lights
        for(auto const spotLight: Light3D->stripeLights) spotLight->setLightDisable();

        // make buttons for 2D Lights
        if (ImGui::CollapsingHeader("TOP"))
        {
            // make buttons for 2D Top Lights
            reset |= ImGui::Checkbox("Visible##DiffuserTOP", &this->diffuserTop->visible);
            reset |= ImGui::Checkbox("Enabled##DiffuserTOP", &this->diffuserTop->enabled);
            reset |= ImGui::SliderFloat("Intensity##DiffuserTOP", &this->diffuserTop->intensity, 0.0f, 200.0f, "%.1f");
            reset |= ImGui::SliderFloat("Height##DiffuserTOP", &this->diffuserTop->height, 0.0f, 10.0f, "%.1f");
            reset |= ImGui::SliderFloat("ringRadius##DiffuserTOP", &this->diffuserTop->ringRadius, 0.0f, 10.0f, "%.1f");
            reset |= ImGui::SliderFloat("ringWidth##DiffuserTOP", &this->diffuserTop->ringWidth, 0.0f, 10.0f, "%.1f");
            reset |= ImGui::Checkbox("DoubleSided##DiffuserTOP", &this->diffuserTop->twosided);
        }

        if (ImGui::CollapsingHeader("Middle"))
        {
            // make buttons for 2D Middle Lights
            reset |= ImGui::SliderInt("Number##DiffuserMiddle", &this->diffuserMiddle->number, 1, 24);
            reset |= ImGui::Checkbox("Visible##DiffuserMiddle", &this->diffuserMiddle->quadLightsVisible);
            reset |= ImGui::Checkbox("Enabled##DiffuserMiddle", &this->diffuserMiddle->quadLightsEnabled);
            reset |= ImGui::SliderFloat("Intensity##DiffuserMiddle", &this->diffuserMiddle->quadLightsIntensity, 0.0f, 100.0f, "%.1f");
            reset |= ImGui::SliderFloat("Height##DiffuserMiddle", &this->diffuserMiddle->height, 0.0f, 10.0f, "%.1f");
            reset |= ImGui::SliderFloat("quadRingRadius##DiffuserMiddle", &this->diffuserMiddle->quadRingRadius, 0.0f, 10.0f, "%.1f");
            reset |= ImGui::SliderFloat("quadWidth##DiffugserMiddle", &this->diffuserMiddle->quadWidth, 0.0f, 10.0f, "%.1f");
            reset |= ImGui::SliderFloat("quadHeight##DiffuserMiddle", &this->diffuserMiddle->quadHeight, 0.0f, 10.0f, "%.1f");
            reset |= ImGui::SliderFloat("tiltedAngle##DiffuserMiddle", &this->diffuserMiddle->quadAngle, 0.0f, 1.57f, "%.1f");
            reset |= ImGui::Checkbox("DoubleSided##DiffuserMiddle", &this->diffuserMiddle->quadTwoSided);
        }

        if (ImGui::CollapsingHeader("BOTTOM"))
        {
            // make buttons for 2D Bottom Lights
            reset |= ImGui::SliderInt("Number##DiffuserBottom", &this->diffuserBottom->number, 1, 24);
            reset |= ImGui::Checkbox("Visible##DiffuserBottom", &this->diffuserBottom->quadLightsVisible);
            reset |= ImGui::Checkbox("Enabled##DiffuserBottom", &this->diffuserBottom->quadLightsEnabled);
            reset |= ImGui::SliderFloat("Intensity##DiffuserBottom", &this->diffuserBottom->quadLightsIntensity, 0.0f, 30.0f, "%.1f");
            reset |= ImGui::SliderFloat("Height##DiffuserBottom", &this->diffuserBottom->height, 0.0f, 10.0f, "%.1f");
            reset |= ImGui::SliderFloat("quadRingRadius##DiffuserBottom", &this->diffuserBottom->quadRingRadius, 0.0f, 10.0f, "%.1f");
            reset |= ImGui::SliderFloat("quadWidth##DiffuserBottom", &this->diffuserBottom->quadWidth, 0.0f, 10.0f, "%.1f");
            reset |= ImGui::SliderFloat("quadHeight##DiffuserBottom", &this->diffuserBottom->quadHeight, 0.0f, 10.0f, "%.1f");
            reset |= ImGui::SliderFloat("tiltedAngle##DiffuserBottom", &this->diffuserBottom->quadAngle, 0.0f, 1.57f, "%.1f");
            reset |= ImGui::Checkbox("DoubleSided##DiffuserBottom", &this->diffuserBottom->quadTwoSided);
        }

    }

    if (ImGui::CollapsingHeader("Heightmap"))
    {
        reset |= ImGui::SliderFloat("LODLevel##Heightmap", &this->currentLod, 0.0f, (float)this->maxMipLevel - 1, "%.1f");
        reset |= ImGui::SliderFloat("RayMarchUnitStep0##Heightmap", &this->rayMarchUnitStep, 0.1f, 1.0f, "%.1f");
        reset |= ImGui::SliderFloat("RayMarchUnitStep1##Heightmap", &this->rayMarchUnitStep, 0.1f, 3.0f, "%.1f");
        reset |= ImGui::SliderFloat("RayMarchUnitStep2##Heightmap", &this->rayMarchUnitStep, 0.1f, 10.0f, "%.1f");
        reset |= ImGui::SliderFloat("RayMarchUnitStep3##Heightmap", &this->rayMarchUnitStep, 0.1f, 30.0f, "%.1f");

        reset |= ImGui::SliderInt("RayMarchMaxStepNumber##Heightmap", &this->rayMarchMaxStepNumber, 100, 1000);

        reset |= ImGui::Checkbox("useNoiseNormalMap##Heightmap", &this->useNoiseNormalMap);
        reset |= ImGui::SliderFloat("normalMapUVCoeff##Heightmap", &this->normalMapUVCoeff, 0.1f, 2.0f, "%.1f");
        reset |= ImGui::SliderFloat("noiseIntensity##Heightmap", &this->noiseIntensity, 0.0f, 1.0f, "%.1f");
    }


    /*if (ImGui::CollapsingHeader("Ring Lights"))
    {
        // make buttons for 2D Lights
        reset |= ImGui::Checkbox("DoubleSidedBottom", &this->diffuserBottom->quadTwoSided);
        reset |= ImGui::Checkbox("DoubleSidedMiddle", &this->diffuserMiddle->quadTwoSided);
    }*/
#endif
}

void SampleRenderer::UpdateLights(){
    VisRTX::Context* context = VisRTX_GetContext();
    // 3D lights
    if(Light3D->number != Light3D->stripeLights.size())
    {
        for(auto stripeLight: Light3D->stripeLights)
        {
            renderer->RemoveLight(stripeLight->spotLight);
        }
        Light3D->stripeLights.clear();
        Light3D->createLights(context);
        for(auto light: Light3D->stripeLights)
        {
            renderer->AddLight(light->spotLight);
            light->Release();
        }
    }
    Light3D->UpdateLight();

    // 2D-top
    diffuserTop->UpdateLight();

    // 2D-middle
    if(diffuserMiddle->number != diffuserMiddle->quadLights.size())
    {
        for(auto light: diffuserMiddle->quadLights)
            renderer->RemoveLight(light);
        diffuserMiddle->quadLights.clear();
        diffuserMiddle->createLights(context);
        for(auto light: diffuserMiddle->quadLights){
            renderer->AddLight(light);
            light->Release();
        }
    }
    diffuserMiddle->UpdateLight();

    // 2D-bottom
    if(diffuserBottom->number != diffuserBottom->quadLights.size())
    {
        for(auto light: diffuserBottom->quadLights)
            renderer->RemoveLight(light);
        diffuserBottom->quadLights.clear();
        diffuserBottom->createLights(context);
        for(auto light: diffuserBottom->quadLights){
            renderer->AddLight(light);
            light->Release();
        }
    }
    this->diffuserBottom->UpdateLight();
}

void SampleRenderer::UpdateCameras(){
    VisRTX::Context* context = VisRTX_GetContext();
    PerspectiveCamera* perspectiveCamera = context->CreatePerspectiveCamera();
    OrthographicCamera* orthographicCamera = context->CreateOrthographicCamera();

    float phi = rotationHorizontal * piOver180;
    float rho = rotationVertical * piOver180;

    float camx = distance * sin(rho) * cos(phi);
    float camy = distance * cos(rho);
    float camz = - distance * sin(rho) * sin(phi);

    Vec3f upVector = cross(Vec3f(1.0f, 0.0f, 0.0f), Vec3f(-camx, -camy, -camz));

    perspectiveCamera->SetPosition(Vec3f(camx, camy, camz));
    perspectiveCamera->SetDirection(Vec3f(-camx, -camy, -camz));
    perspectiveCamera->SetUp(upVector);
    perspectiveCamera->SetFocalDistance(depthOfField ? focalDistance : -1.0f);
    perspectiveCamera->SetApertureRadius(depthOfField ? apertureRadius : -1.0f);
    perspectiveCamera->SetAspect((float)width / (float)height);
    perspectiveCamera->SetImageRegion(this->imageBegin, this->imageEnd);

    orthographicCamera->SetPosition(Vec3f(camx, camy, camz));
    orthographicCamera->SetDirection(Vec3f(-camx, -camy, -camz));
    orthographicCamera->SetUp(upVector);
    orthographicCamera->SetFocalDistance(depthOfField ? focalDistance : -1.0f);
    orthographicCamera->SetApertureRadius(depthOfField ? apertureRadius : -1.0f);
    orthographicCamera->SetHeight(screenHeight);
    orthographicCamera->SetAspect((float)width / (float)height);
    orthographicCamera->SetImageRegion(this->imageBegin, this->imageEnd);

    // camera = 1; // orthographicCamera

    if (camera != lastCamera)
    {

        if (camera == 1)
            renderer->SetCamera(orthographicCamera);
        else
            renderer->SetCamera(perspectiveCamera);

        lastCamera = camera;
        frameNumber = 0;
    }
}

void SampleRenderer::UpdateGeometries(){
    // update lod (for test)
    if(this->heightmapGeo){
        // std::cout << "Height geo update" << std::endl;
        this->heightmapGeo->SetLodLevel(this->currentLod);
        this->heightmapGeo->SetRayMarchStep(this->rayMarchUnitStep, this->rayMarchMaxStepNumber);
        this->heightmapGeo->SetNoiseNormalmap(this->useNoiseNormalMap, this->normalMapUVCoeff, this->noiseIntensity);
    }

    // render the plane only || render plane & models
    if(renderPlaneOnly)
    {
        if(useObj) model->RemoveGeometry(triangleGeo);
        else model->RemoveGeometry(heightmapGeo);
        geometryIsAddedtoModel = false;
    }
    else if(!geometryIsAddedtoModel)
    {
        if(useObj) model->AddGeometry(triangleGeo);
        else model->AddGeometry(heightmapGeo);
        geometryIsAddedtoModel = true;
    }
}

std::string SampleRenderer::GetExportPath()
{
    return "../rendered image/" + this->targetFilePath;
}

void SampleRenderer::Create3DLightImages(bool removeInterreflection)
{
    UpdateCameras();
    UpdateGeometries();

    renderer->SetToneMapping(toneMapping, gamma, colorBalance, whitePoint, burnHighlights, crushBlacks, saturation, brightness);
    renderer->SetDenoiser(aiDenoiser ? DenoiserType::AI : DenoiserType::NONE);
    renderer->SetSamplesPerPixel(samplesPerPixel);
    if(removeInterreflection){
        renderer->SetNumBounces(1, 1);
    } 
    else {
        renderer->SetNumBounces(numBouncesMin, numBouncesMax);
    }
    renderer->SetFireflyClamping(fireflyClampingDirect, fireflyClampingIndirect);
    renderer->SetSampleAllLights(sampleAllLights);

    if (this->clippingPlanesDirty)
    {
        renderer->SetClippingPlanes(this->numClippingPlanes, this->clippingPlanes.data());
        this->clippingPlanesDirty = false;
    }

    // Update light
    ambientLight->SetColor(ambientColor);

    std::string targetName1 = this->GetExportPath() + "/phase_image";
    std::string targetName2 = this->GetExportPath() + "/phase_reference_image";
    
    if(removeInterreflection){
        targetName1.append("_interreflection_removed");
        targetName2.append("_interreflection_removed");
    }

    Create3DLightImagesHelper(targetName1);
    
    renderPlaneOnly = true;
    UpdateGeometries();
    Create3DLightImagesHelper(targetName2);
}

void SampleRenderer::Create2DLightImages(bool removeInterreflection)
{
    UpdateCameras();
    UpdateGeometries();

    renderer->SetToneMapping(toneMapping, gamma, colorBalance, whitePoint, burnHighlights, crushBlacks, saturation, brightness);
    renderer->SetDenoiser(aiDenoiser ? DenoiserType::AI : DenoiserType::NONE);
    renderer->SetSamplesPerPixel(samplesPerPixel);
    if(removeInterreflection){
        renderer->SetNumBounces(1, 1);
    }
    else {
        renderer->SetNumBounces(numBouncesMin, numBouncesMax);
    }
    renderer->SetFireflyClamping(fireflyClampingDirect, fireflyClampingIndirect);
    renderer->SetSampleAllLights(sampleAllLights);

    if (this->clippingPlanesDirty)
    {
        renderer->SetClippingPlanes(this->numClippingPlanes, this->clippingPlanes.data());
        this->clippingPlanesDirty = false;
    }

    // Update light
    ambientLight->SetColor(ambientColor);

    std::string targetName = this->GetExportPath() + "/2D_image";

    if(removeInterreflection){
        targetName.append("_interreflection_removed");
    }

    Create2DLightImagesHelper(targetName);
}

void SampleRenderer::Create3DLightImagesHelper(std::string dirname)
{
    makePath(dirname);

    for(int i=0; i<Light3D->number; i++){
        // disable 3D Lights
        for(auto const spotLight: Light3D->stripeLights) spotLight->setLightDisable();
        // disable 2D Lights
        this->diffuserTop->setLightDisable();
        this->diffuserMiddle->setLightDisable();
        this->diffuserBottom->setLightDisable();

        Light3D->stripeLights[i]->setLightEnable();

        for(int phase=1; phase<=4; phase++){
            Light3D->stripeLights[i]->currentlyONPhase = phase;
            this->UpdateLights();

            for(int frame_number = 0; frame_number < this->exportImageFrameNumber; frame_number++){
                renderer->Render(frameBuffer);
            }
            
            int imageIndex = i * 4 + phase;
            this->frameBuffer->ScreenShot(dirname + "/3DImage_"
                        + std::to_string(imageIndex) + ".png");
            frameBuffer->Clear();
        }
    }
}

void SampleRenderer::Create2DLightImagesHelper(std::string dirname)
{
    makePath(dirname);
    for(int light=1; light <= KohyoungConstant::NUM_2DLIGHT; light++)
    {
        this->diffuserTop->setLightDisable();
        this->diffuserMiddle->setLightDisable();
        this->diffuserBottom->setLightDisable();

        std::string imageName;
        if(light == KohyoungConstant::TOP)
        {
            this->diffuserTop->setLightEnable();
            imageName = std::string("top");
        }
        else if(light == KohyoungConstant::MIDDLE)
        {
            this->diffuserMiddle->setLightEnable();
            imageName = std::string("mid");
        }
        else if(light == KohyoungConstant::BOTTOM)
        {
            this->diffuserBottom->setLightEnable();
            imageName = std::string("bot");
        }
        else return;

        this->UpdateLights();
        int exportImageFrameNumber = this->exportImageFrameNumber;
        for(int frame_number = 0; frame_number < exportImageFrameNumber; frame_number++){
            renderer->Render(frameBuffer);
        }


        this->frameBuffer->ScreenShot(dirname + "/" + imageName + ".png");
        frameBuffer->Clear();
    }
}

void SampleRenderer::Run(const std::string& title, int argc, char **argv)
{
    try
    {
#ifdef VISRTX_DYNLOAD
        // Load library first
        if (!VisRTX_LoadLibrary())
        {
            std::cerr << "Error: Failed to load VisRTX library" << std::endl;
            return;
        }
#endif
        // Device query and selection
        VisRTX::Context* context = VisRTX_GetContext();
        if (!context)
        {
            std::cerr << "Error: No capable device found (OptiX 6 requires SM 5.0 / Maxwell and up)" << std::endl;
            return;
        }

        // Command line args
        bool benchmark = false;
        bool offscreen = false;
        bool egl = false;
        bool useOpenGL = false;
        bool dump = false;
        for (int i = 0; i < argc; ++i)
        {
            if (std::string(argv[i]) == "benchmark")
            {
                benchmark = true;
            }
            else if (std::string(argv[i]) == "offscreen")
            {
                offscreen = true;
            }
            else if (std::string(argv[i]) == "egl")
            {
#ifdef VISRTX_SAMPLE_WITH_EGL
                egl = true;
                offscreen = true;
#else
                std::cout << "Error: EGL not enabled. Build VisRTX with VISRTX_SAMPLE_WITH_EGL=ON." << std::endl;
#endif
            }
            else if (std::string(argv[i]) == "dump")
            {
                dump = true;
            }
        }

        if (benchmark)
        {
            std::cout << "--- VisRTX Benchmark ---" << std::endl;
            std::cout << "Resolution: " << width << " x " << height << std::endl;
            std::cout << "Ray bounces: " << numBouncesMin << " - " << numBouncesMax << std::endl;
            std::cout << "Samples per pixel: " << samplesPerPixel << std::endl;
            std::cout << std::endl;
        }

        PerspectiveCamera* perspectiveCamera = context->CreatePerspectiveCamera();
        OrthographicCamera* orthographicCamera = context->CreateOrthographicCamera();

        model = context->CreateModel();

        renderer = context->CreateRenderer();
        renderer->SetModel(model);

        // Ambient light
        ambientLight = context->CreateAmbientLight();
        renderer->AddLight(ambientLight);

#ifdef VISRTX_SAMPLE_WITH_GLFW
        // Create window
        if (!offscreen)
        {
            glfwSetErrorCallback(glfwErrorCallback);

            if (!glfwInit())
            {
                std::cerr << "Error: Failed to initialize GLFW." << std::endl;
                return;
            }

            const char* glsl_version = "#version 450";
            glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
            glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);

            std::string fullTitle = benchmark ? (title + " - Benchmark") : title;
            window = glfwCreateWindow(width, height, fullTitle.c_str(), NULL, NULL);
            if (window == nullptr)
            {
                std::cerr << "Error: Failed to create window." << std::endl;
                return;
            }

            glfwSetWindowUserPointer(window, this);

            glfwMakeContextCurrent(window);
            glfwSwapInterval(0); // no vsync

            gladLoadGLLoader((GLADloadproc) glfwGetProcAddress);

            glfwSetKeyCallback(window, keyCallback);
            glfwSetMouseButtonCallback(window, mouseButtonCallback);
            glfwSetCursorPosCallback(window, cursorCallback);


            // Setup Dear ImGui binding
            IMGUI_CHECKVERSION();
            ImGui::CreateContext();
            ImGuiIO& io = ImGui::GetIO();
            io.FontAllowUserScaling = true;
            io.FontGlobalScale = 2.0f;

            ImGui_ImplGlfw_InitForOpenGL(window, true);
            ImGui_ImplOpenGL3_Init(glsl_version);

            ImGui::StyleColorsDark();

            useOpenGL = true;
        }
#endif


#ifdef VISRTX_SAMPLE_WITH_EGL
        if (offscreen && egl)
        {
            display = eglGetDisplay(EGL_DEFAULT_DISPLAY);

            EGLint major, minor;
            if (eglInitialize(display, &major, &minor))
                std::cout << "EGL: YES" << std::endl;
            else
                std::cerr << "Error: Failed to initialize EGL" << std::endl;

            const EGLint configAttribs[] = {
                EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
                EGL_BLUE_SIZE, 8,
                EGL_GREEN_SIZE, 8,
                EGL_RED_SIZE, 8,
                EGL_DEPTH_SIZE, 8,
                EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
                EGL_NONE
            };

            EGLint numConfigs;
            EGLConfig config;
            eglChooseConfig(display, configAttribs, &config, 1, &numConfigs);

            const EGLint pbufferAttribs[] = {
                EGL_WIDTH, width,
                EGL_HEIGHT, height,
                EGL_NONE,
            };

            surface = eglCreatePbufferSurface(display, config, pbufferAttribs);

            eglBindAPI(EGL_OPENGL_API);
            EGLContext context = eglCreateContext(display, config, EGL_NO_CONTEXT, NULL);
            eglMakeCurrent(display, surface, surface, context);

            gladLoadGLLoader((GLADloadproc) eglGetProcAddress);

            useOpenGL = true;
        }
#endif

        // Init OpenGL
        if(useOpenGL)
        {
            glClearColor(0.0f, 1.0f, 0.0f, 0.0f);

            // Create shader and texture for fullscreen display of decompressed frame
            const GLchar* clientVertexShader =
                    "#version 330\n"
                    "void main() {}";

            const GLchar* clientGeometryShader =
                    "#version 330 core\n"
                    "layout(points) in;"
                    "layout(triangle_strip, max_vertices = 4) out;"
                    "out vec2 texcoord;"
                    "void main() {"
                    "gl_Position = vec4( 1.0, 1.0, 0.0, 1.0 ); texcoord = vec2( 1.0, 1.0 ); EmitVertex();"
                    "gl_Position = vec4(-1.0, 1.0, 0.0, 1.0 ); texcoord = vec2( 0.0, 1.0 ); EmitVertex();"
                    "gl_Position = vec4( 1.0,-1.0, 0.0, 1.0 ); texcoord = vec2( 1.0, 0.0 ); EmitVertex();"
                    "gl_Position = vec4(-1.0,-1.0, 0.0, 1.0 ); texcoord = vec2( 0.0, 0.0 ); EmitVertex();"
                    "EndPrimitive();"
                    "}";

            const GLchar* clientFragmentShader =
                    "#version 330\n"
                    "uniform sampler2D tex;"
                    "uniform float useGray;"
                    "in vec2 texcoord;"
                    "out vec4 color;"
                    "void main() {"
                    "   color = texture(tex, texcoord);"
                    "   if(useGray > 0.5f)"
                    "   {"
                    "       vec3 grayCoeff = vec3(0.299, 0.587, 0.114);"
                    "       float gray = dot(grayCoeff, color.rgb);"
                    "       color.rgb = vec3(gray);"
                    "   }"
                    "}";

            GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
            glShaderSource(vertexShader, 1, &clientVertexShader, 0);
            glCompileShader(vertexShader);

            GLuint geometryShader = glCreateShader(GL_GEOMETRY_SHADER);
            glShaderSource(geometryShader, 1, &clientGeometryShader, 0);
            glCompileShader(geometryShader);

            GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
            glShaderSource(fragmentShader, 1, &clientFragmentShader, 0);
            glCompileShader(fragmentShader);

            fullscreenQuadProgram = glCreateProgram();
            glAttachShader(fullscreenQuadProgram, vertexShader);
            glAttachShader(fullscreenQuadProgram, geometryShader);
            glAttachShader(fullscreenQuadProgram, fragmentShader);
            glLinkProgram(fullscreenQuadProgram);

            fullscreenTextureLocation = glGetUniformLocation(fullscreenQuadProgram, "tex");
            useGrayLocation = glGetUniformLocation(fullscreenQuadProgram, "useGray");

            glGenVertexArrays(1, &fullscreenVAO);
        }
        frameBuffer = context->CreateFrameBuffer(VisRTX::FrameBufferFormat::RGBA8);

        // Init sample
        if (!this->Init(argc, argv))
            escapePressed = true;

        /*
         * Main loop
         */
        rotateTimer.Reset();
        fpsTimer.Reset();

        Timer benchmarkTimer;
        uint32_t benchmarkFrame = 0;
        const uint32_t benchmarkWarmupFrames = 100;
        const uint32_t benchmarkDynamicFrames = 1000;
        const uint32_t benchmarkStaticFrames = 1000;

        const float benchmarkTimeDelta = 1.0f / 60.0f; // 60 Hz animation speed target

        double dynamicTime = 0.0;
        double staticTime = 0.0;

        BenchmarkPhase benchmarkPhase = INIT;
        time_t start, end;
        start = time(NULL);
        if(exportImage3D){
           this->Create3DLightImages(this->exportImageRemoveInterreflection);
            end = time(NULL);
            printf("3D export time: %f\n", (double)(end-start));
           return;
        }
        else if(exportImage2D){

            this->Create2DLightImages(this->exportImageRemoveInterreflection);
            end = time(NULL);
            printf("2D export time: %f\n", (double)(end-start));
            return;
        }

        while (!escapePressed)
        {
#ifdef VISRTX_SAMPLE_WITH_GLFW
            if (!offscreen && glfwWindowShouldClose(window))
                break;
#endif
            // Benchmark update
            if (benchmark)
            {
                BenchmarkPhase lastPhase = benchmarkPhase;

                if (benchmarkFrame < benchmarkWarmupFrames)
                    benchmarkPhase = WARMUP;
                else if (benchmarkFrame < benchmarkWarmupFrames + benchmarkDynamicFrames)
                    benchmarkPhase = DYNAMIC;
                else if (benchmarkFrame < benchmarkWarmupFrames + benchmarkDynamicFrames + benchmarkStaticFrames)
                    benchmarkPhase = STATIC;
                else
                    benchmarkPhase = COMPLETE;

                // Phase changed
                if (benchmarkPhase != lastPhase)
                {
                    double elapsed = benchmarkTimer.GetElapsedSeconds();

                    if (lastPhase == DYNAMIC)
                        dynamicTime = elapsed;
                    else if (lastPhase == STATIC)
                        staticTime = elapsed;

                    if (benchmarkPhase == WARMUP)
                        std::cout << "Warmup..." << std::endl;
                    else if (benchmarkPhase == DYNAMIC)
                        std::cout << "Dynamic..." << std::endl;
                    else if (benchmarkPhase == STATIC)
                        std::cout << "Static..." << std::endl;

                    lastPhase = benchmarkPhase;

                    benchmarkTimer.Reset();
                }

                if (benchmarkPhase == COMPLETE)
                {
                    std::cout << std::endl;

                    auto printPerf = [](const std::string& title, uint32_t numFrames, double elapsedSeconds)
                    {
                        double frameRate = numFrames / elapsedSeconds;
                        double frameTime = elapsedSeconds * 1000.0 / numFrames;
                        std::cout << title << ": " << std::setprecision(1) << std::fixed << frameRate << " Hz (" << frameTime << " ms)" << std::endl;
                    };

                    printPerf("Dynamic", benchmarkDynamicFrames, dynamicTime);
                    printPerf("Static", benchmarkStaticFrames, staticTime);

                    // Dump final frame onyl
                    if (dump)
                        this->DumpFrame(useOpenGL);

                    break;
                }


                ++benchmarkFrame;
            }

            // Begin frame
#ifdef VISRTX_SAMPLE_WITH_GLFW
            if (!offscreen)
            {

                glfwMakeContextCurrent(window);

                if (!benchmark)
                {
                    ImGui_ImplOpenGL3_NewFrame();
                    ImGui_ImplGlfw_NewFrame();
                    ImGui::NewFrame();
                }

                int w, h;
                glfwGetFramebufferSize(window, &w, &h);
                if (w != width || h != height)
                {
                    frameNumber = 0;
                    width = w;
                    height = h;
                }
            }
#endif

#ifdef VISRTX_SAMPLE_WITH_EGL
            if (offscreen && egl)
            {
                //eglMakeCurrent(display, surface, surface, context);
            }
#endif
            if (useOpenGL)
            {
                glViewport(0, 0, width, height);
				glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            }

            // Update camera
            if (!benchmark && autoRotate && !pauseAllAnimations)
            {
                rotationHorizontal += rotateSpeed * (float)rotateTimer.GetElapsedSeconds();
                frameNumber = 0;
            }
            rotateTimer.Reset();

            float phi = rotationHorizontal * piOver180;
            float rho = rotationVertical * piOver180;

            float camx = distance * sin(rho) * cos(phi);
            float camy = distance * cos(rho);
            float camz = - distance * sin(rho) * sin(phi);

            Vec3f upVector = cross(Vec3f(1.0f, 0.0f, 0.0f), Vec3f(-camx, -camy, -camz));

            perspectiveCamera->SetPosition(Vec3f(camx, camy, camz));
            perspectiveCamera->SetDirection(Vec3f(-camx, -camy, -camz));
            perspectiveCamera->SetUp(upVector);
            perspectiveCamera->SetFocalDistance(depthOfField ? focalDistance : -1.0f);
            perspectiveCamera->SetApertureRadius(depthOfField ? apertureRadius : -1.0f);
            perspectiveCamera->SetAspect((float)width / (float)height);
			perspectiveCamera->SetImageRegion(this->imageBegin, this->imageEnd);

            orthographicCamera->SetPosition(Vec3f(camx, camy, camz));
            orthographicCamera->SetDirection(Vec3f(-camx, -camy, -camz));
            orthographicCamera->SetUp(upVector);
            orthographicCamera->SetFocalDistance(depthOfField ? focalDistance : -1.0f);
            orthographicCamera->SetApertureRadius(depthOfField ? apertureRadius : -1.0f);
            orthographicCamera->SetHeight(screenHeight);
            orthographicCamera->SetAspect((float)width / (float)height);
			orthographicCamera->SetImageRegion(this->imageBegin, this->imageEnd);

            // camera = 1; // orthographicCamera

            if (camera != lastCamera)
            {

                if (camera == 1)
                    renderer->SetCamera(orthographicCamera);
                else
                    renderer->SetCamera(perspectiveCamera);

                lastCamera = camera;
                frameNumber = 0;
            }

            // Update render settings
            renderer->SetToneMapping(toneMapping, gamma, colorBalance, whitePoint, burnHighlights, crushBlacks, saturation, brightness);
            renderer->SetDenoiser(aiDenoiser ? DenoiserType::AI : DenoiserType::NONE);
            renderer->SetSamplesPerPixel(samplesPerPixel);
            renderer->SetNumBounces(numBouncesMin, numBouncesMax);
            renderer->SetFireflyClamping(fireflyClampingDirect, fireflyClampingIndirect);
            renderer->SetSampleAllLights(sampleAllLights);

            if (this->clippingPlanesDirty)
            {
                renderer->SetClippingPlanes(this->numClippingPlanes, this->clippingPlanes.data());
                this->clippingPlanesDirty = false;
            }

            // Update light
            ambientLight->SetColor(ambientColor);

            // Update sample
            bool reset = false;
            this->UpdateScene(benchmark, benchmarkPhase, benchmarkTimeDelta, pauseAllAnimations, reset);

            if (!progressiveRendering)
                reset = true;

            if (reset)
                frameNumber = 0;

            // Render
            frameBuffer->Resize(VisRTX::Vec2ui(width, height));
            if (frameNumber == 0)
                frameBuffer->Clear();

            this->renderTimer.Reset();
            renderer->Render(frameBuffer);
            this->renderTime += this->renderTimer.GetElapsedMilliseconds();
            ++this->renderTimeCounter;


            // Display image
            if (useOpenGL)
            {
                this->displayTimer.Reset();

				if (frameNumber == 0 || this->updateGLTimer.GetElapsedMilliseconds() >= this->updateGLInterval)
				{
					this->updateGLTimer.Reset();
					this->colorTex = frameBuffer->GetColorTextureGL();
				}

				glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
				glUseProgram(fullscreenQuadProgram);
				glActiveTexture(GL_TEXTURE0);
				glBindTexture(GL_TEXTURE_2D, this->colorTex);
				glUniform1i(fullscreenTextureLocation, 0);
                if(useGray) glUniform1f(useGrayLocation, 1.0f);
                else    glUniform1f(useGrayLocation, 0.0f);
				glBindVertexArray(fullscreenVAO);
				glDrawArrays(GL_POINTS, 0, 1);

                this->displayTime += this->displayTimer.GetElapsedMilliseconds();
                ++this->displayTimeCounter;
            }

            // GUI
            reset = false;
            if (!benchmark)
            {
                ++fpsCounter;
                const float elapsed = (float)fpsTimer.GetElapsedSeconds();
                if (elapsed >= fpsUpdateInterval)
                {
                    fps = (float)fpsCounter / elapsed;
                    fpsCounter = 0;
                    fpsTimer.Reset();

                    this->renderTimeAverage = this->renderTimeCounter > 0 ? (this->renderTime / this->renderTimeCounter) : 0.0;
                    this->displayTimeAverage = this->displayTimeCounter > 0 ? (this->displayTime / this->displayTimeCounter) : 0.0;

                    this->renderTime = 0.0;
                    this->displayTime = 0.0;

                    this->renderTimeCounter = 0;
                    this->displayTimeCounter = 0;
                }

                if (fpsPrintTimer.GetElapsedSeconds() > 3.0f)
                {
                    std::cout << std::fixed << std::setprecision(1) << fps << " Hz (" << (fps > 0.0f ? 1000.0f / fps : 0.0f) << " ms)" << std::endl;
                    fpsPrintTimer.Reset();
                }

#ifdef VISRTX_SAMPLE_WITH_GLFW
                if (useOpenGL && !offscreen)
                {
                    if (showGUI)
                    {
                        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiSetCond_FirstUseEver);
                        ImGui::Begin("Settings", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings);

                        ImGui::Value("Frame Number", (int)frameNumber);
                        ImGui::Text("Resolution  : %u x %u", width, height);
                        ImGui::Text("Frame Rate  : %.1f Hz (%.1f ms)", fps, fps > 0.0f ? 1000.0f / fps : 0.0f);
                        ImGui::Text(" - Render   : %.1f ms", this->renderTimeAverage);
                        ImGui::Text(" - Display  : %.1f ms", this->displayTimeAverage);
                        ImGui::Separator();

                        ImGui::Checkbox("Progressive Rendering", &progressiveRendering);
                        ImGui::Checkbox("Pause All Animations", &pauseAllAnimations);
						ImGui::SliderFloat("##Update GL Interval", &this->updateGLInterval, 0.0f, 1000.0f, "Display every %.0f ms");
                        ImGui::Spacing();

                        if (ImGui::CollapsingHeader("Renderer"))
                        {
                            reset |= ImGui::SliderInt("Samples", &samplesPerPixel, 1, 32);
                            reset |= ImGui::SliderInt("Min Bounces", &numBouncesMin, 0, 10);
                            reset |= ImGui::SliderInt("Max Bounces", &numBouncesMax, 0, 50);
                            reset |= ImGui::SliderFloat("Clamping Direct", &fireflyClampingDirect, 0.0f, 1000.0f);
                            reset |= ImGui::SliderFloat("Clamping Indirect", &fireflyClampingIndirect, 0.0f, 1000.0f);
                            reset |= ImGui::Checkbox("Sample All Lights", &sampleAllLights);
                            ImGui::Separator();
                            reset |= ImGui::Checkbox("AI Denoiser", &aiDenoiser);
                        }

                        ImGui::Spacing();

                        if (ImGui::CollapsingHeader("Tone Mapping"))
                        {
                            reset |= ImGui::Checkbox("Enabled", &toneMapping);
                            reset |= ImGui::ColorEdit3("Color Balance", (float*)&colorBalance);
                            reset |= ImGui::SliderFloat("Gamma", &gamma, 0.01f, 10.0f); // Must not get 0.0f
                            reset |= ImGui::SliderFloat("White Point", &whitePoint, 0.01f, 255.0f, "%.2f", 2.0f); // Must not get 0.0f
                            reset |= ImGui::SliderFloat("Burn Hightlights", &burnHighlights, 0.0f, 10.0f, "%.2f");
                            reset |= ImGui::SliderFloat("Crush Blacks", &crushBlacks, 0.0f, 1.0f, "%.2f");
                            reset |= ImGui::SliderFloat("Saturation", &saturation, 0.0f, 10.0f, "%.2f");
                            reset |= ImGui::SliderFloat("Brightness", &brightness, 0.0f, 100.0f, "%.2f", 2.0f);
                        }

                        ImGui::Spacing();

                        if (ImGui::CollapsingHeader("Camera"))
                        {


                            if(camera == 0) // perspective
                            {
                                ImGui::Text("Distance from the plane");
                                reset |= ImGui::SliderFloat("DistanceLevel0##Camera", &distance, 0.0f, 1000.0f);
                                reset |= ImGui::SliderFloat("DistanceLevel1##Camera", &distance, 0.0f, 100.0f);
                                reset |= ImGui::SliderFloat("DistanceLevel2##Camera", &distance, 0.0f, 10.0f);
                                reset |= ImGui::SliderFloat("DistanceLevel3##Camera", &distance, 0.0f, 1.0f);
                                reset |= ImGui::SliderFloat("DistanceLevel4##Camera", &distance, 0.0f, 0.1f);
                                reset |= ImGui::SliderFloat("DistanceLevel5##Camera", &distance, 0.0f, 0.01f);
                                ImGui::Text(" ");
                            }
                            else // orthographic(telecentric)
                            {
                                ImGui::Text("Distance from the plane");
                                reset |= ImGui::SliderFloat("DistanceLevel0##Camera", &distance, 0.0f, 1000.0f);
                                reset |= ImGui::SliderFloat("DistanceLevel1##Camera", &distance, 0.0f, 100.0f);
                                reset |= ImGui::SliderFloat("DistanceLevel2##Camera", &distance, 0.0f, 10.0f);
                                reset |= ImGui::SliderFloat("DistanceLevel3##Camera", &distance, 0.0f, 1.0f);
                                reset |= ImGui::SliderFloat("DistanceLevel4##Camera", &distance, 0.0f, 0.1f);
                                reset |= ImGui::SliderFloat("DistanceLevel5##Camera", &distance, 0.0f, 0.01f);
                                ImGui::Text("Screen Height");
                                reset |= ImGui::SliderFloat("ScreenHeightLevel0##Camera", &screenHeight, 0.0f, 100.0f);
                                reset |= ImGui::SliderFloat("ScreenHeightLevel1##Camera", &screenHeight, 0.0f, 10.0f);
                                reset |= ImGui::SliderFloat("ScreenHeightLevel2##Camera", &screenHeight, 0.0f, 1.0f);
                                reset |= ImGui::SliderFloat("ScreenHeightLevel3##Camera", &screenHeight, 0.0f, 0.1f);
                                reset |= ImGui::SliderFloat("ScreenHeightLevel4##Camera", &screenHeight, 0.0f, 0.01f);
                                ImGui::Text(" ");
                            }

                            reset |= ImGui::SliderFloat("rotation_Vertical##Camera", &rotationVertical, -90.0f, 90.0f);
                            ImGui::RadioButton("Perspective", &camera, 0); ImGui::SameLine();
                            ImGui::RadioButton("Orthographic(Telecentric)", &camera, 1);

                            // Dof also for Orthographic --> telecentric
                            //if (camera == 0)
                            //{
                            reset |= ImGui::Checkbox("Depth of Field", &depthOfField);
                            if (depthOfField)
                            {
                                reset |= ImGui::SliderFloat("Focal Distance", &focalDistance, 0.0f, 2.0f * distance, "%.02f");
                                reset |= ImGui::SliderFloat("Aperture", &apertureRadius, 0.0f, 3.0f, "%.3f");
                                if(apertureRadius > 0.0001 && focalDistance > 0.0001)
                                {
                                    ImGui::Text("F Number: %.1f", focalDistance / (apertureRadius * 2.0));
                                    ImGui::Text("NA: %.3f", apertureRadius / focalDistance);
                                }
                            }
                            //}

							ImGui::Separator();

							reset |= ImGui::SliderFloat2("Begin", &this->imageBegin.x, 0.0f, 1.0f, "%.2f");
							reset |= ImGui::SliderFloat2("End", &this->imageEnd.x, 0.0f, 1.0f, "%.2f");
                        }

                        ImGui::Spacing();

                        if (ImGui::CollapsingHeader("Ambient Light"))
                        {
                            reset |= ImGui::ColorEdit3("Color", &ambientColor.x);
                        }

                        ImGui::Spacing();

                        this->UpdateGUI(reset);

                        if(ImGui::CollapsingHeader("Models"))
                        {
                            // plane only mode
                            reset |= ImGui::Checkbox("removeModels", &this->renderPlaneOnly);

                            if (!this->materials.empty())
                            {
                                ImGui::Spacing();
                                ImGui::Text("MDL Materials");
                                ImGui::Spacing();

                                for (MaterialGUI& mat : this->materials)
                                {
                                    std::string matTitle = std::string(mat.mdl->GetName());
                                    if (!mat.objectName.empty())
                                        matTitle += " (" + mat.objectName + ")";
                                    matTitle += "##" + std::to_string(mat.id);

                                    if (ImGui::CollapsingHeader(matTitle.c_str()))
                                    {
                                        for (ParameterGUI& p : mat.parameters)
                                        {
                                            const std::string title = p.name + "##" + std::to_string(mat.id);

                                            if (p.type == VisRTX::ParameterType::FLOAT)
                                            {
                                                if (ImGui::DragFloat(title.c_str(), &p.valueFloat, 0.001f))
                                                {
                                                    mat.mdl->SetParameterFloat(p.name.c_str(), p.valueFloat);
                                                    reset = mat.mdl->GetCompilationType() == VisRTX::CompilationType::CLASS;
                                                    mat.dirty = true;
                                                }
                                            }
                                            else if (p.type == VisRTX::ParameterType::DOUBLE)
                                            {
                                                if (ImGui::DragFloat(title.c_str(), &p.valueFloat, 0.001f))
                                                {
                                                    mat.mdl->SetParameterDouble(p.name.c_str(), p.valueFloat);
                                                    reset = mat.mdl->GetCompilationType() == VisRTX::CompilationType::CLASS;
                                                    mat.dirty = true;
                                                }
                                            }
                                            else if (p.type == VisRTX::ParameterType::INT)
                                            {
                                                if (ImGui::DragInt(title.c_str(), &p.valueInt))
                                                {
                                                    mat.mdl->SetParameterInt(p.name.c_str(), p.valueInt);
                                                    reset = mat.mdl->GetCompilationType() == VisRTX::CompilationType::CLASS;
                                                    mat.dirty = true;
                                                }
                                            }
                                            else if (p.type == VisRTX::ParameterType::BOOL)
                                            {
                                                if (ImGui::Checkbox(title.c_str(), &p.valueBool))
                                                {
                                                    mat.mdl->SetParameterBool(p.name.c_str(), p.valueBool);
                                                    reset = mat.mdl->GetCompilationType() == VisRTX::CompilationType::CLASS;
                                                    mat.dirty = true;
                                                }
                                            }
                                            else if (p.type == VisRTX::ParameterType::COLOR)
                                            {
                                                if (ImGui::ColorEdit3(title.c_str(), &p.valueColor.r, ImGuiColorEditFlags_Float | ImGuiColorEditFlags_HDR))
                                                {
                                                    mat.mdl->SetParameterColor(p.name.c_str(), p.valueColor);
                                                    reset = mat.mdl->GetCompilationType() == VisRTX::CompilationType::CLASS;
                                                    mat.dirty = true;
                                                }
                                            }
                                        }

                                        // Recompile button
                                        if (mat.mdl->GetCompilationType() == VisRTX::CompilationType::INSTANCE && mat.dirty)
                                        {
                                            ImGui::PushStyleColor(0, ImVec4(1.0f, 1.0f, 0.0f, 1.0f));

                                            const std::string buttonStr = "Compile##" + std::to_string(mat.id);
                                            if (ImGui::Button(buttonStr.c_str()))
                                            {
                                                mat.mdl->Compile();
                                                reset = true;
                                                mat.dirty = false;
                                            }

                                            ImGui::PopStyleColor();
                                        }
                                    }
                                }
                            }
                        }



                        ImGui::End();
                    }

                    ImGui::Render();
                    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
                }
#endif
            }

            // End frame
            if (reset)
                frameNumber = 0;
            else
                ++frameNumber;

            // Dump frame
            if (dump && !benchmark)
                this->DumpFrame(useOpenGL);

#ifdef VISRTX_SAMPLE_WITH_GLFW
            if (!offscreen)
            {
                glfwSwapBuffers(window);
                glfwPollEvents();
            }

#endif

#ifdef VISRTX_SAMPLE_WITH_EGL
            if (offscreen && egl)
            {
                eglSwapBuffers(display, surface);
            }
#endif
        }

        // Clean up
        for (VisRTX::Object* obj : this->releaseLater)
            obj->Release();


        ambientLight->Release();

        perspectiveCamera->Release();
        orthographicCamera->Release();
        renderer->Release();
        model->Release();
        frameBuffer->Release();

#ifdef VISRTX_SAMPLE_WITH_GLFW
        if (!offscreen)
        {
            ImGui_ImplOpenGL3_Shutdown();
            ImGui_ImplGlfw_Shutdown();
            ImGui::DestroyContext();

            glfwDestroyWindow(window);
            glfwTerminate();
        }
#endif

#ifdef VISRTX_SAMPLE_WITH_EGL
        if (offscreen && egl)
        {
            eglTerminate(display);
        }
#endif


    }
    catch (VisRTX::Exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return;
    }
}
