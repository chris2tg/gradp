#ifndef LIGHT_SIMULATOR_H
#define LIGHT_SIMULATOR_H
#include "Sample.h"
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

class LightSimulator{
public:
    virtual void UpdateLight()=0;
    virtual void setLightEnable()=0;
    virtual void setLightDisable()=0;
};

class stripeLight
{
public:
    int name;
    float distance, rotationHorizontal, rotationVertical;
    float rotationHorizontalOffset;
    Vec3f wPosition;
    SpotLight* spotLight;
    bool spotLightEnabled;
    bool spotLightVisible;
    float spotLightIntensity;
    float spotLightRadius;
    float spotLightOpeningAngle;
    float spotLightPenumbraAngle;
    float angularWavenumber;
    float angularOffset;
    float phaseLineAngle;
    int currentlyONPhase; // 1 ~ 4


    stripeLight(Context* context): name(-1), distance(30.0f),
                        rotationHorizontal(0.0f), rotationVertical(0.0f), rotationHorizontalOffset(0.0f),
                        spotLight(NULL), spotLightEnabled(false), spotLightVisible(false),spotLightIntensity(5500.0f),

                        spotLightRadius(0.0f), spotLightOpeningAngle(12.0f), spotLightPenumbraAngle(0.0f),
                        angularWavenumber(18000.0f), angularOffset(0.0f), phaseLineAngle(0.0f), currentlyONPhase(1) // for structured light

    {
        spotLight = context->CreateSpotLight();
    }

    void setRotationHorizontalOffset(float _rotationHorizontalOffset)
    {
        rotationHorizontalOffset = _rotationHorizontalOffset;
    }

    void setPosition(float _distance, float rotationHorizontal_degree, float rotationVertical_degree)
    {
        distance = _distance;
        rotationHorizontal = rotationHorizontal_degree;
        rotationVertical = rotationVertical_degree;

        float phi = rotationHorizontal * piOver180;
        float rho = rotationVertical * piOver180;

        float Light_x = distance * sin(rho) * cos(phi);
        float Light_y = distance * cos(rho);
        float Light_z = - distance * sin(rho) * sin(phi);

        wPosition = Vec3f(Light_x, Light_y, Light_z);
        spotLight->SetColor(Vec3f(1.0f, 1.0f, 1.0f));
        spotLight->SetPosition(wPosition);
        spotLight->SetDirection(Vec3f(-Light_x, -Light_y, -Light_z));
    }

    void SetPhaseLineAngle(float phaseLineAngle)
    {
        this->phaseLineAngle = phaseLineAngle;
    }

    void SetStructuredLight(float angularWavenumber, float angularOffset, float phaseLineAngle)
    {
        this->angularWavenumber = angularWavenumber;
        this->angularOffset = angularOffset;
        this->phaseLineAngle = phaseLineAngle;
    }

    void SetAngularWavenumber(float angularWavenumber){
        this->angularWavenumber = angularWavenumber;
    }

    virtual void UpdateLight()
    {
        float phi = (rotationHorizontal + rotationHorizontalOffset) * piOver180;
        float rho = rotationVertical * piOver180;

        float Light_x = distance * sin(rho) * cos(phi);
        float Light_y = distance * cos(rho);
        float Light_z = - distance * sin(rho) * sin(phi);

        // [hui2007]
        // 3D Profile Reconstruction of Solder Paste Based on Phase Shift Profilometry
        angularOffset = (float)(currentlyONPhase - 1) / 2.0f * M_PI;

        wPosition = Vec3f(Light_x, Light_y, Light_z);
        spotLight->SetColor(Vec3f(1.0f, 1.0f, 1.0f));
        spotLight->SetPosition(wPosition);
        spotLight->SetDirection(Vec3f(-Light_x, -Light_y, -Light_z));
        spotLight->SetEnabled(spotLightEnabled);
        spotLight->SetVisible(spotLightVisible);
        spotLight->SetIntensity(spotLightIntensity);
        spotLight->SetRadius(spotLightRadius);
        spotLight->SetOpeningAngle(spotLightOpeningAngle);
        spotLight->SetPenumbraAngle(spotLightPenumbraAngle);
        spotLight->SetStructuredLight(angularWavenumber, angularOffset, phaseLineAngle);
    }

    void setLightEnable()
    {
        spotLightEnabled = true;
    } 

    void setLightDisable()
    {
        spotLightEnabled = false;
    }

    void Release()
    {
        spotLight->Release();
    }
};

class LightSimulator_3D: public LightSimulator
{
public:
    int number;
    float defaultPhaseLineAngle;
    float rotationHorizontalOffset;
    float DISTANCE_3D_LIGHTS;
    float ELEVATION_ANGLE_3D_LIGHTS; // degree
    float DIFFUSE_ANGLE_3D_LIGHTS; // degree
    float WAVE_LENGTH_EVEN; // cm
    float WAVE_LENGTH_ODD; // cm
    std::vector<stripeLight*> stripeLights;
    LightSimulator_3D(Context* context)
    {
        number = 8;
        defaultPhaseLineAngle = 0.0f;
        rotationHorizontalOffset = 0.0f;
    }

    void createLights(Context* context){
        float distance_3DLight = DISTANCE_3D_LIGHTS;
        float rotationHorizontal_temp = 135.0f;
        float phaseLineAngle_temp = 135.0f + 90.0f; // orthogonal phase
        float rotationVertical_temp = 90.0f - ELEVATION_ANGLE_3D_LIGHTS;

        for(int iter = 1; iter <= number; iter++)
        {
            stripeLight* newLight = new stripeLight(context);
            newLight->name = iter;
            newLight->setPosition(distance_3DLight, rotationHorizontal_temp, rotationVertical_temp);
            float wavelengthOnBase = (iter%2==0 ? WAVE_LENGTH_EVEN : WAVE_LENGTH_ODD);
            wavelengthOnBase *= cos(rotationVertical_temp * piOver180);
            float unitWavelength = wavelengthOnBase / distance_3DLight;
            float unitWavenumber = 2 * pi / unitWavelength;
            newLight->SetStructuredLight(unitWavenumber, 0.f, (defaultPhaseLineAngle + phaseLineAngle_temp)*piOver180);
            newLight->spotLightOpeningAngle = DIFFUSE_ANGLE_3D_LIGHTS;
            stripeLights.push_back(newLight);

            rotationHorizontal_temp += 360.0f / float(number);
            phaseLineAngle_temp -= 360.0f / float(number);
        }
    }

    virtual void UpdateLight()
    {
        float phaseLineAngle_temp = 135.0f + 90.0f; // orthogonal phase
        for(int i=0; i<stripeLights.size(); i++)
        {
            stripeLights[i]->SetPhaseLineAngle((defaultPhaseLineAngle + phaseLineAngle_temp)*piOver180);
            stripeLights[i]->setRotationHorizontalOffset(rotationHorizontalOffset);
            stripeLights[i]->UpdateLight();
            phaseLineAngle_temp -= 360.0f / float(number);
        }
    }

    virtual void setLightEnable()
    {
        return;
    }

    virtual void setLightDisable()
    {
        for(int i=0; i<stripeLights.size(); i++)
        {
            stripeLight *strLight = stripeLights[i];
            strLight->setLightDisable();
        }
    }

};

class LightSimulator_DiffuserQuadRing: public LightSimulator
{
public:
    int number;
    float height;
    std::vector<QuadLight*> quadLights;
    bool quadLightsEnabled;
    bool quadLightsVisible;
    bool quadTwoSided;
    float quadLightsIntensity;
    float quadRingRadius;
    float quadWidth, quadHeight;
    float quadAngle;
    Vec3f lightColor;
    LightSimulator_DiffuserQuadRing(Context* context)
    {
        number = 8;
        height = 1;
        quadLightsEnabled = true;
        quadLightsVisible = true;
        quadTwoSided = false;
        quadLightsIntensity = 3.0f;
        quadRingRadius = 3.0f;
        quadWidth = 2.0f;
        quadHeight = 1.0f;
        quadAngle = atan(height / quadRingRadius);
        quadLights = std::vector<QuadLight*>();
        lightColor = Vec3f(0,0,0);
    }

    void createLights(Context* context){
        for(int i=0; i<number; i++){
            QuadLight* quadLight = context->CreateQuadLight();

            glm::vec3 right = glm::vec3(0, 0, 1);
            glm::vec3 up = glm::vec3(0, 1, 0);
            glm::vec3 pos = glm::vec3(quadRingRadius, 0, 0);

            glm::mat4 trans = glm::mat4(1.0f);
            trans = glm::rotate(trans, glm::radians(i/(float)number * 360), glm::vec3(0.0, 1.0, 0.0));
            trans = glm::rotate(trans, quadAngle, glm::vec3(0.0, 0.0, 1.0));

            right = glm::mat3(trans) * right * quadWidth;
            up = glm::mat3(trans) * up * quadHeight;

            trans = glm::mat4(1.0f);
            trans = glm::rotate(trans, glm::radians(i/(float)number * 360), glm::vec3(0.0, 1.0, 0.0));

            pos = glm::mat3(trans) * pos;

            pos = pos - up * 0.5f - right * 0.5f;

            Vec3f anchor = Vec3f(pos.x, pos.y + height, pos.z);
            Vec3f edge1 = Vec3f(right.x, right.y, right.z);
            Vec3f edge2 = Vec3f(up.x, up.y, up.z);
            
            quadLight->SetRect(anchor, edge1, edge2);
            quadLights.push_back(quadLight);
        }
        UpdateLight();
    }

    void release(){
        for(int i=0; i<quadLights.size(); i++){
            QuadLight* quadLight = quadLights[i];
            quadLight->Release();
        }
    }

    virtual void UpdateLight() {
        for (int i = 0; i < quadLights.size(); i++) {
            QuadLight *quadLight = quadLights[i];

            // quadLight pos & size update
            glm::vec3 right = glm::vec3(0, 0, 1);
            glm::vec3 up = glm::vec3(0, 1, 0);
            glm::vec3 pos = glm::vec3(quadRingRadius, 0, 0);

            glm::mat4 trans = glm::mat4(1.0f);
            trans = glm::rotate(trans, glm::radians(i/(float)quadLights.size() * 360), glm::vec3(0.0, 1.0, 0.0));
            trans = glm::rotate(trans, quadAngle, glm::vec3(0.0, 0.0, 1.0));

            right = glm::mat3(trans) * right * quadWidth;
            up = glm::mat3(trans) * up * quadHeight;

            trans = glm::mat4(1.0f);
            trans = glm::rotate(trans, glm::radians(i/(float)quadLights.size() * 360), glm::vec3(0.0, 1.0, 0.0));

            pos = glm::mat3(trans) * pos;

            pos = pos - up * 0.5f - right * 0.5f;

            Vec3f anchor = Vec3f(pos.x, pos.y + height, pos.z);
            Vec3f edge1 = Vec3f(right.x, right.y, right.z);
            Vec3f edge2 = Vec3f(up.x, up.y, up.z);

            quadLight->SetRect(anchor, edge1, edge2);

            // other setting
            quadLight->SetColor(lightColor);
            quadLight->SetEnabled(quadLightsEnabled);
            quadLight->SetVisible(quadLightsVisible);
            quadLight->SetIntensity(quadLightsIntensity);
            quadLight->SetTwoSided(quadTwoSided);
        }
    }

    virtual void setLightEnable()
    {
        quadLightsEnabled = true;
    } 

    virtual void setLightDisable()
    {
        quadLightsEnabled = false;
    }
};

class LightSimulator_DiffuserRing: public LightSimulator{
public:
    RingLight* ringLight;
    float height;
    float ringRadius;
    float ringWidth;
    float intensity;
    Vec3f lightColor;
    bool enabled;
    bool visible;
    bool twosided;
    LightSimulator_DiffuserRing(Context* context)
    {
        height = 1;
        enabled = true;
        visible = true;
        twosided = false;
        intensity = 3.0f;
    }

    void createLights(Context* context){
        this->ringLight = context->CreateRingLight();
        this->ringLight->SetPosition(Vec3f(0, height, 0));
        this->ringLight->SetRadii(Vec2f(ringRadius+ringWidth*0.5f, ringRadius-ringWidth*0.5f));
        this->ringLight->SetColor(lightColor);
        UpdateLight();
    }

    void release(){
        ringLight->Release();
    }

    virtual void UpdateLight()
    {
        ringLight->SetEnabled(enabled);
        ringLight->SetIntensity(intensity);
        ringLight->SetVisible(visible);
        ringLight->SetPosition(Vec3f(0, height, 0));
        ringLight->SetRadii(Vec2f(ringRadius+ringWidth*0.5f, ringRadius-ringWidth*0.5f));
        ringLight->SetTwoSided(this->twosided);
    }

    virtual void setLightEnable()
    {
        enabled = true;
    } 

    virtual void setLightDisable()
    {
        enabled = false;
    }
};
#endif