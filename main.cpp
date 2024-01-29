#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <complex>
#include <chrono>
#include <memory>

using namespace std::chrono;
using namespace std;

struct ShaderProgramSource
{
    string VertexSource;
    string FragmentSource;
};

static ShaderProgramSource ParseShader(const string& filepath)
{
    ifstream stream(filepath);

    enum class ShaderType
    {
        NONE = -1, VERTEX = 0, FRAGMENT = 1
    };

    string line;
    stringstream ss[2];
    ShaderType type = ShaderType::NONE;
    while (getline(stream, line))
    {
        if (line.find("#shader") != string::npos)
        {
            if (line.find("vertex") != string::npos)
                type = ShaderType::VERTEX;
            else if (line.find("fragment") != string::npos)
                type = ShaderType::FRAGMENT;
        }
        else
        {
            ss[(int)type] << line << '\n';
        }
    }
    return { ss[0].str(), ss[1].str() };
}

static unsigned int CompileShader(unsigned int type, const string& source)
{
    unsigned int id = glCreateShader(type);
    const char* src = source.c_str();
    glShaderSource(id, 1, &src, nullptr);
    glCompileShader(id);

    int result;
    glGetShaderiv(id, GL_COMPILE_STATUS, &result);
    if (result == GL_FALSE)
    {
        int length;
        glGetShaderiv(id, GL_INFO_LOG_LENGTH, &length);
        char* message = (char*)alloca(length * sizeof(char));
        glGetShaderInfoLog(id, length, &length, message);
        cout << "Failed to compile " << (type == GL_VERTEX_SHADER ? "vertex" : "fragment") << " shader!" << endl;
        cout << message << endl;
        glDeleteShader(id);
        return 0;
    }
    return id;
}

static int CreateShader(const string& vertexShader, const string& fragmentShader)
{
    unsigned int program = glCreateProgram();
    unsigned int vs = CompileShader(GL_VERTEX_SHADER, vertexShader);
    unsigned int fs = CompileShader(GL_FRAGMENT_SHADER, fragmentShader);

    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);
    glValidateProgram(program);

    glDeleteShader(vs);
    glDeleteShader(fs);

    return program;
}

const int width = 960;
const int height = 960;
const float M_PI = 3.14159265358979323846;
const float M_PI2 = 2 * M_PI;
const int PointsPerCircle = 50;


unsigned int circleIndicesSize;
unsigned int* circleIndices;

struct vec2
{
    float x, y;

    vec2(){}

    vec2(float xi, float yi)
    {
        x = xi;
        y = yi;
    }

    vec2 operator +(vec2 v)
    {
        return vec2(x + v.x, y + v.y);
    }

    vec2 operator -(vec2 v)
    {
        return vec2(x - v.x, y - v.y);
    }

    vec2 operator *(vec2 v)
    {
        return vec2(x * v.x, y * v.y);
    }

    vec2 operator /(vec2 v)
    {
        return vec2(x / v.x, y / v.y);
    }

    vec2 operator *(float c)
    {
        return vec2(x * c, y * c);
    }

    vec2 operator /(float c)
    {
        return vec2(x / c, y / c);
    }

    void operator +=(vec2 v)
    {
        x += v.x;
        y += v.y;
    }

    float norm()
    {
        return sqrt(x * x + y * y);
    }
};

vec2 normalize(vec2 v)
{
    float norm = v.norm();
    v.x /= norm;
    v.y /= norm;
    return v;
}

float crossProduct(vec2 p1, vec2 p2)
{
    return p1.x * p2.y - p1.y * p2.x;
}

float dot(vec2 p1, vec2 p2)
{
    return p1.x * p2.x + p1.y * p2.y;
}

void CreateCircleVertices(vec2 center, float radius, vec2 *vertices)
{
    vertices[0] = center;
    float theta = 0;
    float thetaStep = M_PI2/(PointsPerCircle - 1);
    for (int i = 0; i < PointsPerCircle; i++)
    {
        vertices[i + 1] = center + vec2(cos(theta), sin(theta)) * radius;
        theta += thetaStep;
    }
}

unsigned int* CreateCircleIndices(unsigned int& size)
{
    size = 3 * PointsPerCircle;
    unsigned int* indices = new unsigned int[size];
    for (int i = 0; i < PointsPerCircle - 1; i++)
    {
        indices[i * 3] = 0;
        indices[i * 3 + 1] = i + 1;
        indices[i * 3 + 2] = i + 2;
    }
    indices[(PointsPerCircle - 1) * 3] = 0;
    indices[(PointsPerCircle - 1) * 3 + 1] = PointsPerCircle;
    indices[(PointsPerCircle - 1) * 3 + 2] = 1;
    return indices;
}

void Translate(vec2* positions, unsigned int size, vec2 translation)
{
    for (int i = 0; i < size; i++)
    {
        positions[i] += translation;
    }
}

class obstacle
{
public:
    int verticesNeeded = 0;
    int indicesNeeded = 0;
    virtual  bool insideObstacle(vec2 point) { return false; }
    virtual void createVertices(vec2* arr) {}
    virtual void createIndices(unsigned int* arr, int indexOffset) {}
};

int triangleIndices[] = { 0, 1, 2 };

class triangle : public obstacle
{
public:
    vec2* vertices;
    vec2* edges;
    triangle(vec2 p1, vec2 p2, vec2 p3)
    {
        verticesNeeded = 3;
        indicesNeeded = 3;
        vertices = new vec2[3]{ p1, p2, p3 };
        edges = new vec2[3]{
            vertices[1] - vertices[0],
            vertices[2] - vertices[1],
            vertices[0] - vertices[2]
        };
    }

    bool insideObstacle(vec2 point)
    {
        float ABxAP = crossProduct(edges[0], point - vertices[0]);
        float BCxBP = crossProduct(edges[1], point - vertices[1]);
        float CAxCP = crossProduct(edges[2], point - vertices[2]);

        return ((ABxAP < 0 && BCxBP < 0 && CAxCP < 0) || (ABxAP > 0 && BCxBP > 0 && CAxCP > 0));
    }

    void createVertices(vec2* arr) override
    {
        for (int i = 0; i < 3; i++)
        {
            arr[i] = vertices[i];
        }
    }

    void createIndices(unsigned int* arr, int indexOffset)
    {
        for (int i = 0; i < 3; i++)
        {
            arr[i] = indexOffset + i;
        }
    }
};

class circle : public obstacle
{
public:
    vec2 center;
    float radius;
    circle(vec2 c, float r)
    {
        verticesNeeded = PointsPerCircle + 1;
        indicesNeeded = PointsPerCircle * 3;
        center = c;
        radius = r;
    }

    bool insideObstacle(vec2 point)
    {
        return (point - center).norm() < radius;
    }

    void createVertices(vec2* arr)
    {
        CreateCircleVertices(center, radius, arr);
    }

    void createIndices(unsigned int* arr, int indexOffset)
    {
        for (int i = 0; i < circleIndicesSize; i++)
        {
            arr[i] = circleIndices[i] + indexOffset;
        }
    }
};

struct obstacleWorld
{
    vec2* vertices;
    unsigned int* indices;
    unsigned int verticesSize;
    unsigned int indicesSize;

    obstacleWorld(vector<obstacle*> obstacleList)
    {
        verticesSize = 0;
        indicesSize = 0;
        for (const auto &currObstacle : obstacleList)
        {
            verticesSize += currObstacle -> verticesNeeded;
            indicesSize += currObstacle -> indicesNeeded;
        }

        vertices = new vec2[verticesSize];
        indices = new unsigned int[indicesSize];
        int curriVertex = 0;
        int curriIndex = 0;

        for (auto &currObstacle : obstacleList)
        {
            currObstacle -> createVertices(&vertices[curriVertex]);
            currObstacle -> createIndices(&indices[curriIndex], curriVertex);
            curriVertex += currObstacle -> verticesNeeded;
            curriIndex += currObstacle -> indicesNeeded;
        }
    }
};

vec2 robotCenter(0, 1);
float robotRadius = 0.02;
float robotSpeed = 0.01;
float robotVisionRadius = 0.1;
vector<obstacle*> obstacleList;
vec2 goalCenter(0, -1);
float goalRadius = 0.02;
float angleStep = .01;
vec2 movingTowards = robotCenter;
bool followingBorder = false;
float raySpeed = robotSpeed / 4;

bool raycast(vec2 origin, vec2 direction, float r, vector<obstacle*> &obstacleList, vec2 &hitPoint)
{
    hitPoint = origin;
    vec2 step = direction * raySpeed;
    for (int i = 0; i < r / raySpeed; i++)
    {
        hitPoint += step;
        for (auto obs : obstacleList)
        {
            if (obs -> insideObstacle(hitPoint))
                return true;
        }
    }
    return false;
}

void circleCast(vec2 origin, float r, vector<obstacle*> &obstacleList, vector<vec2> &hitPoints)
{
    vec2 hitPoint;
    vec2 lastHitPoint;
    bool hittingObstacle = raycast(origin, vec2(1, 0), r, obstacleList, hitPoint);
    for (float angle = angleStep; angle < M_PI2; angle += angleStep)
    {
        if (raycast(origin, vec2(cos(angle), sin(angle)), r, obstacleList, hitPoint))
        {
            if(!hittingObstacle)
                hitPoints.push_back(lastHitPoint);
            hittingObstacle = true;
            lastHitPoint = hitPoint;
        }
        else if (hittingObstacle)
        {
            hitPoints.push_back(hitPoint);
            hittingObstacle = false;
        }
    }
}

void moveRobot(vec2 direction, float speed, vec2 *robotVertices, int robotVertexAmount)
{

}

int main(void)
{
    cv::utils::logging::setLogLevel(cv::utils::logging::LogLevel::LOG_LEVEL_SILENT);

    GLFWwindow* window;

    /* Initialize the library */
    if (!glfwInit())
        return -1;


    /* Create a windowed mode window and its OpenGL context */
    window = glfwCreateWindow(width, height, "Tangent Bug", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        return -1;
    }

    /* Make the window's context current */
    glfwMakeContextCurrent(window);

    glfwSwapInterval(1);

    if (glewInit() != GLEW_OK)
        cout << "Error!" << endl;

    float screenPositions[] =
    {
        -1.0f, -1.0f,
         1.0f, -1.0f,
         1.0f,  1.0f,
        -1.0f,  1.0f
    };

    unsigned int screenIndices[] =
    {
        0, 1, 2,
        2, 3, 0
    };

    circleIndices = CreateCircleIndices(circleIndicesSize);

    vec2* robotVisionPositions = new vec2[PointsPerCircle + 1];
    CreateCircleVertices(robotCenter, robotVisionRadius, robotVisionPositions);

    vec2* robotPositions = new vec2[PointsPerCircle + 1];
    CreateCircleVertices(robotCenter, robotRadius, robotPositions);

    vec2* movingTowardsPositions = new vec2[PointsPerCircle + 1];
    CreateCircleVertices(movingTowards, robotRadius, movingTowardsPositions);

    vec2* goalPositions = new vec2[PointsPerCircle + 1];
    CreateCircleVertices(goalCenter, goalRadius, goalPositions);

    obstacleList.push_back(new circle(
        vec2(0, 0.5), .3
    ));

    obstacleList.push_back(new circle(
        vec2(0, -0.5), .3
    ));

    //obstacleList.push_back(triangle(
    //    vec2(1, -1),
    //    vec2(0.3, -0.4),
    //    vec2(1, 0)));


    obstacleWorld world(obstacleList);

    unsigned int screenBuffer;
    unsigned int robotBuffer;
    unsigned int goalBuffer;
    unsigned int obstacleBuffer;
    unsigned int robotVisionBuffer;
    unsigned int movingTowardsBuffer;
    unsigned int obstacleIbo;
    unsigned int screenIbo;
    unsigned int circleIbo;

    glGenBuffers(1, &screenBuffer);
    glGenBuffers(1, &robotBuffer);
    glGenBuffers(1, &goalBuffer);
    glGenBuffers(1, &obstacleBuffer);
    glGenBuffers(1, &robotVisionBuffer);
    glGenBuffers(1, &movingTowardsBuffer);
    glGenBuffers(1, &screenIbo);
    glGenBuffers(1, &circleIbo);
    glGenBuffers(1, &obstacleIbo);

    glBindBuffer(GL_ARRAY_BUFFER, screenBuffer);
    glBufferData(GL_ARRAY_BUFFER, 8 * sizeof(float), screenPositions, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, screenIbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, 6 * sizeof(unsigned int), screenIndices, GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, robotBuffer);
    glBufferData(GL_ARRAY_BUFFER, (PointsPerCircle + 1) * sizeof(vec2), robotPositions, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, circleIbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, circleIndicesSize * sizeof(unsigned int), circleIndices, GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, goalBuffer);
    glBufferData(GL_ARRAY_BUFFER, (PointsPerCircle + 1) * sizeof(vec2), goalPositions, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, 0);

    glBindBuffer(GL_ARRAY_BUFFER, obstacleBuffer);
    glBufferData(GL_ARRAY_BUFFER, world.verticesSize * sizeof(vec2), world.vertices, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, obstacleIbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, world.indicesSize * sizeof(unsigned int), world.indices, GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, robotVisionBuffer);
    glBufferData(GL_ARRAY_BUFFER, (PointsPerCircle + 1) * sizeof(vec2), robotVisionPositions, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, 0);

    glBindBuffer(GL_ARRAY_BUFFER, movingTowardsBuffer);
    glBufferData(GL_ARRAY_BUFFER, (PointsPerCircle + 1) * sizeof(vec2), movingTowardsPositions, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, 0);


    unsigned int fbo, render_buf;
    glGenFramebuffers(1, &fbo);
    glGenRenderbuffers(1, &render_buf);
    glBindRenderbuffer(GL_RENDERBUFFER, render_buf);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGB, width, height);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo);
    glFramebufferRenderbuffer(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, render_buf);

    ShaderProgramSource source = ParseShader("Basic.shader");

    unsigned int shader = CreateShader(source.VertexSource, source.FragmentSource);
    glUseProgram(shader);

    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

    int inputColLocation = glGetUniformLocation(shader, "inputCol");
    if (inputColLocation == -1) return -1;

    vec2 newCenter;
    vec2 translation;

    vec2 goalDirection = normalize(goalCenter - robotCenter);

    float distToGoal = (goalCenter - robotCenter).norm();
    float lastDistToGoal = 999999999;
    float dreach = 999999999;
    float dfollowed = 999999999;
    vec2 lastDirection;
    vec2 pointToFollow;
    vec2 followingBoundaryStartingPos;

    cv::Mat M;
    M.create(height, width, CV_8UC3);

    cv::VideoWriter outputVideo;
    int codec = cv::VideoWriter::fourcc('m', 'p', '4', 'v');

    if (!outputVideo.open("VideoTangentBug.mp4", codec, 60.0, M.size(), true))
    {
        cout << "Problema al abrir el archivo" << endl;
        return -1;
    }

    //glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo);

    bool done = false;

    /* Loop until the user closes the window */
    while (!glfwWindowShouldClose(window) && !done)
    {
        /* Render here */
        glClear(GL_COLOR_BUFFER_BIT);

        //Draw world
        glUniform3f(inputColLocation, 0, 0, 1);
        glBindBuffer(GL_ARRAY_BUFFER, screenBuffer);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, screenIbo);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);

        //Draw vision
        glUniform3f(inputColLocation, 1, 1, 0);
        glBindBuffer(GL_ARRAY_BUFFER, robotVisionBuffer);
        glBufferData(GL_ARRAY_BUFFER, (PointsPerCircle + 1) * sizeof(vec2), robotVisionPositions, GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, circleIbo);
        glDrawElements(GL_TRIANGLES, circleIndicesSize, GL_UNSIGNED_INT, nullptr);

        //Draw obstacles
        glUniform3f(inputColLocation, 0, 0, 0);
        glBindBuffer(GL_ARRAY_BUFFER, obstacleBuffer);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, obstacleIbo);
        glDrawElements(GL_TRIANGLES, world.indicesSize, GL_UNSIGNED_INT, nullptr);

        //Draw goal
        glUniform3f(inputColLocation, 0, 1, 0);
        glBindBuffer(GL_ARRAY_BUFFER, goalBuffer);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, circleIbo);
        glDrawElements(GL_TRIANGLES, circleIndicesSize, GL_UNSIGNED_INT, nullptr);

        //Draw moving to point
        glUniform3f(inputColLocation, 0.5, 0.5, 0.5);
        glBindBuffer(GL_ARRAY_BUFFER, movingTowardsBuffer);
        glBufferData(GL_ARRAY_BUFFER, (PointsPerCircle + 1) * sizeof(vec2), movingTowardsPositions, GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, circleIbo);
        glDrawElements(GL_TRIANGLES, circleIndicesSize, GL_UNSIGNED_INT, nullptr);

        //Draw robot
        glUniform3f(inputColLocation, 1, 0, 0);
        glBindBuffer(GL_ARRAY_BUFFER, robotBuffer);
        glBufferData(GL_ARRAY_BUFFER, (PointsPerCircle + 1) * sizeof(vec2), robotPositions, GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, circleIbo);
        glDrawElements(GL_TRIANGLES, circleIndicesSize, GL_UNSIGNED_INT, nullptr);

        //Update robot
        lastDistToGoal = distToGoal;
        if (distToGoal <= robotSpeed)
        {
            translation = goalCenter - robotCenter;
            robotCenter = goalCenter;
            Translate(robotPositions, (PointsPerCircle + 1), translation);
            Translate(robotVisionPositions, (PointsPerCircle + 1), translation);
            done = true;
        }
        else
        {
            vec2 raycastHit;
            if (!followingBorder && !raycast(robotCenter, goalDirection, robotVisionRadius, obstacleList, raycastHit))
            {
                vec2 movingTowardsTranslation = robotCenter + goalDirection * robotVisionRadius - movingTowards;
                movingTowards = robotCenter + goalDirection * robotVisionRadius;
                translation = goalDirection * robotSpeed;
                robotCenter += translation;
                Translate(robotPositions, (PointsPerCircle + 1), translation);
                Translate(robotVisionPositions, (PointsPerCircle + 1), translation);
                Translate(movingTowardsPositions, (PointsPerCircle + 1), movingTowardsTranslation);
            }
            else
            {
                vector<vec2> pointsToFollow;
                circleCast(robotCenter, robotVisionRadius, obstacleList, pointsToFollow);
                float minDist = 9999999999;
                for (auto& point : pointsToFollow)
                {
                    if (followingBorder && dot(normalize(point - robotCenter), lastDirection) < -.01)
                        continue;

                    float distToPoint = (point - robotCenter).norm();
                    float distPointToGoal = (goalCenter - point).norm();
                    if (distPointToGoal + distToPoint < minDist)
                    {
                        pointToFollow = point;
                        minDist = distToGoal + distToPoint;
                        dreach = distPointToGoal;
                    }
                }
                vec2 movingTowardsTranslation = pointToFollow - movingTowards;
                movingTowards = pointToFollow;
                Translate(movingTowardsPositions, (PointsPerCircle + 1), movingTowardsTranslation);
                translation = normalize(pointToFollow - robotCenter) * robotSpeed;
                robotCenter += translation;
                Translate(robotPositions, (PointsPerCircle + 1), translation);
                Translate(robotVisionPositions, (PointsPerCircle + 1), translation);
                goalDirection = normalize(goalCenter - robotCenter);
            }
        }
        distToGoal = (goalCenter - robotCenter).norm();
        lastDirection = normalize(pointToFollow - robotCenter);
        if (!followingBorder && distToGoal > lastDistToGoal)
        {
            dfollowed = (goalCenter - pointToFollow).norm();
            followingBorder = true;
            followingBoundaryStartingPos = robotCenter;
        }

        if (followingBorder && dreach < dfollowed)
        {
            followingBorder = false;
        }
        //glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo);
        //glReadBuffer(GL_COLOR_ATTACHMENT0);
        //glReadPixels(0, 0, width, height, GL_BGR, GL_UNSIGNED_BYTE, M.data);

        //cv::flip(M, M, 0);

        //outputVideo << M;

        /* Swap front and back buffers */
        glfwSwapBuffers(window);

        /* Poll for and process events */
        glfwPollEvents();
    }

    glDeleteProgram(shader);

    glfwTerminate();

    return 0;
}