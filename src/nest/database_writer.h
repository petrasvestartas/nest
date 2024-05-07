// Notes:
// Serialization: The serializeSQLPolyline function converts a SQLPolyline object into a serialized string format, ready for storage in the SQLite database.
// Database Initialization: The program starts by opening (and creating if necessary) the SQLite database polylines.db. It then ensures the SQLPolylines table exists by executing the CREATE TABLE IF NOT EXISTS SQL statement.
// Data Insertion: The insertSQLPolylines function iterates over a vector of SQLPolyline objects, serializes each one, and inserts the serialized data into the database along with the polyline color.
// Program Execution: Running this program will create the polylines.db SQLite database file in the current working directory (if it doesn't already exist), create the SQLPolylines table within that database, and insert two sample polyline records.
// cmake -S . -B build -G "Visual Studio 17 2022" -A x64 -DCMAKE_BUILD_TYPE=Release && cmake --build build --config Release --parallel 8 && build\Release\opengl_viewer
#include <sqlite3.h>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include "../stdafx.h"

// Represents a polyline with a flat vector of coordinates and a color.
struct SQLPolyline
{
    std::vector<float> vertices; // Flat vector of vertex coordinates: x, y, z, ...
    std::string color;           // Hexadecimal color code, e.g., "#FF0000" for red.
    float thickness = 10.0f;     // Thickness of the polyline.
};

// Represents a mesh with vertices, normals, and indices.
struct SQLMesh
{
    std::vector<float> vertices; // Flat vector of vertex coordinates.
    std::vector<float> normals;  // Flat vector of normal coordinates.
    std::vector<int> indices;    // Flat vector of triangle face indices.
    std::string color;           // Hexadecimal color code.
};

// Represents a point cloud with vertices, normals, and colors.
struct SQLPointCloud
{
    std::vector<float> vertices; // Flat vector of vertex coordinates: x, y, z, ...
    std::vector<float> normals;  // Flat vector of normal coordinates: nx, ny, nz, ...
    std::vector<float> colors;   // Flat vector of colors: r, g, b, ...
};

inline std::string serializeSQLPolyline(const SQLPolyline &polyline)
{
    std::ostringstream ss;
    for (size_t i = 0; i < polyline.vertices.size(); i += 3)
    {
        ss << std::fixed << std::setprecision(6)
           << polyline.vertices[i] << ","
           << polyline.vertices[i + 1] << ","
           << polyline.vertices[i + 2] << ";";
    }
    return ss.str();
}

inline std::string serializeSQLMesh(const SQLMesh &mesh)
{
    std::ostringstream ss;
    for (size_t i = 0; i < mesh.vertices.size(); i += 3)
    {
        ss << std::fixed << std::setprecision(6)
           << mesh.vertices[i] << ","
           << mesh.vertices[i + 1] << ","
           << mesh.vertices[i + 2] << ";";
    }
    ss << "|";
    for (size_t i = 0; i < mesh.normals.size(); i += 3)
    {
        ss << std::fixed << std::setprecision(6)
           << mesh.normals[i] << ","
           << mesh.normals[i + 1] << ","
           << mesh.normals[i + 2] << ";";
    }
    ss << "|";
    for (size_t i = 0; i < mesh.indices.size(); i += 3)
    {
        ss << mesh.indices[i] << ","
           << mesh.indices[i + 1] << ","
           << mesh.indices[i + 2] << ";";
    }
    return ss.str();
}

inline std::string serializeSQLPointCloud(const SQLPointCloud &pointCloud)
{
    std::ostringstream ss;
    for (size_t i = 0; i < pointCloud.vertices.size(); i += 3)
    {
        ss << std::fixed << std::setprecision(6)
           << pointCloud.vertices[i] << ","
           << pointCloud.vertices[i + 1] << ","
           << pointCloud.vertices[i + 2] << ";";
    }
    ss << "|";
    for (size_t i = 0; i < pointCloud.normals.size(); i += 3)
    {
        ss << std::fixed << std::setprecision(6)
           << pointCloud.normals[i] << ","
           << pointCloud.normals[i + 1] << ","
           << pointCloud.normals[i + 2] << ";";
    }
    ss << "|";
    for (size_t i = 0; i < pointCloud.colors.size(); i += 3)
    {
        ss << std::fixed << std::setprecision(6)
           << pointCloud.colors[i] << ","
           << pointCloud.colors[i + 1] << ","
           << pointCloud.colors[i + 2] << ";";
    }
    return ss.str();
}

inline void execSql(sqlite3 *db, const std::string &sql)
{
    char *errMsg = nullptr;
    int rc = sqlite3_exec(db, sql.c_str(), nullptr, nullptr, &errMsg);
    if (rc != SQLITE_OK)
    {
        std::cerr << "SQL error: " << errMsg << std::endl;
        sqlite3_free(errMsg);
    }
}

inline void insertSQLPolylines(sqlite3 *db, const std::vector<SQLPolyline> &polylines, bool clearDbFirst = false)
{
    if (clearDbFirst)
    {
        execSql(db, "DELETE FROM SQLPolylines;");
    }

    const char *sql = "INSERT INTO SQLPolylines (Data, Color, Thickness) VALUES (?, ?, ?);";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, NULL) != SQLITE_OK)
    {
        std::cerr << "SQL error: " << sqlite3_errmsg(db) << std::endl;
        return;
    }

    for (const auto &polyline : polylines)
    {
        std::string data = serializeSQLPolyline(polyline);

        // Bind the serialized data
        sqlite3_bind_text(stmt, 1, data.c_str(), -1, SQLITE_TRANSIENT);

        // Bind the color
        sqlite3_bind_text(stmt, 2, polyline.color.c_str(), -1, SQLITE_TRANSIENT);

        // Bind the thickness as a REAL
        sqlite3_bind_double(stmt, 3, static_cast<double>(polyline.thickness));

        if (sqlite3_step(stmt) != SQLITE_DONE)
        {
            std::cerr << "Error inserting data: " << sqlite3_errmsg(db) << std::endl;
        }

        // Reset the statement to be reused
        sqlite3_reset(stmt);
    }

    // Finalize the statement to free resources
    sqlite3_finalize(stmt);
}

inline void insertSQLMeshes(sqlite3 *db, const std::vector<SQLMesh> &meshes, bool clearDbFirst = false)
{
    if (clearDbFirst)
    {
        execSql(db, "DELETE FROM SQLMeshes;");
    }

    for (const auto &mesh : meshes)
    {
        std::string data = serializeSQLMesh(mesh);
        execSql(db, "INSERT INTO SQLMeshes (Data, Color) VALUES ('" + data + "', '" + mesh.color + "');");
    }
}

inline void insertSQLPointClouds(sqlite3 *db, const std::vector<SQLPointCloud> &pointClouds, bool clearDbFirst = false)
{
    if (clearDbFirst)
    {
        execSql(db, "DELETE FROM SQLPointClouds;");
    }

    for (const auto &pointCloud : pointClouds)
    {
        std::string data = serializeSQLPointCloud(pointCloud);
        execSql(db, "INSERT INTO SQLPointClouds (Data) VALUES ('" + data + "');");
    }
}

inline void write_example() {
    sqlite3 *db;
    if (sqlite3_open("C:/brg/2_code/database_viewer/cmake/src/viewer/database/database_viewer.db", &db) == SQLITE_OK)
    {
        // Enable WAL mode
        sqlite3_exec(db, "PRAGMA journal_mode=WAL;", 0, 0, 0);

        // Ensure the tables exist and are properly structured.
        // Continue with the rest of your database operations...
    }
    else
    {
        std::cerr << "Can't open database: " << sqlite3_errmsg(db) << std::endl;
        return;
    }

    // Ensure the tables exist and are properly structured.

    execSql(db, "CREATE TABLE IF NOT EXISTS SQLPolylines (ID INTEGER PRIMARY KEY AUTOINCREMENT, Data TEXT NOT NULL, Color TEXT NOT NULL, Thickness REAL NOT NULL);");
    execSql(db, "CREATE TABLE IF NOT EXISTS SQLMeshes (ID INTEGER PRIMARY KEY AUTOINCREMENT, Data TEXT NOT NULL, Color TEXT NOT NULL);");
    execSql(db, "CREATE TABLE IF NOT EXISTS SQLPointClouds (ID INTEGER PRIMARY KEY AUTOINCREMENT, Data TEXT NOT NULL);");

    // Example data for polylines, meshes, and point clouds
    std::vector<SQLPolyline> exampleSQLPolylines = {
        {{
             -0.5f,
             -0.5f,
             0.5f,
             0.5f,
             -0.5f,
             0.5f,
             0.5f,
             0.5f,
             0.5f,
             -0.5f,
             0.5f,
             0.5f,
             -0.5f,
             -0.5f,
             0.5f,
         },
         "#098FCC",
         3.0f},
        {{
             0.5f,
             -0.5f,
             -0.5f,
             -0.5f,
             -0.5f,
             -0.5f,
             -0.5f,
             0.5f,
             -0.5f,
             0.5f,
             0.5f,
             -0.5f,
             0.5f,
             -0.5f,
             -0.5f,
         },
         "#eb0b7c",
         3.0f}};

    std::vector<SQLMesh> exampleSQLMeshes = {
        {// Vertices: Correctly ordered for counter-clockwise winding
         {
             // Front face (CCW from the front)
             -0.5f, -0.5f, 0.5f,
             0.5f, -0.5f, 0.5f,
             0.5f, 0.5f, 0.5f,
             -0.5f, 0.5f, 0.5f,
             // Back face (CCW from the front)
             0.5f, -0.5f, -0.5f,
             -0.5f, -0.5f, -0.5f,
             -0.5f, 0.5f, -0.5f,
             0.5f, 0.5f, -0.5f,
             // Top face (CCW from the top)
             -0.5f, 0.5f, -0.5f,
             -0.5f, 0.5f, 0.5f,
             0.5f, 0.5f, 0.5f,
             0.5f, 0.5f, -0.5f,
             // Bottom face (CCW from the top)
             -0.5f, -0.5f, -0.5f,
             0.5f, -0.5f, -0.5f,
             0.5f, -0.5f, 0.5f,
             -0.5f, -0.5f, 0.5f,
             // Right face (CCW from the right)
             0.5f, -0.5f, 0.5f,
             0.5f, -0.5f, -0.5f,
             0.5f, 0.5f, -0.5f,
             0.5f, 0.5f, 0.5f,
             // Left face (CCW from the right)
             -0.5f, -0.5f, -0.5f,
             -0.5f, -0.5f, 0.5f,
             -0.5f, 0.5f, 0.5f,
             -0.5f, 0.5f, -0.5f},
         // Normals for each vertex
         {
             // Front face
             0.0f, 0.0f, 10.0f,
             0.0f, 0.0f, 1.0f,
             0.0f, 0.0f, 1.0f,
             0.0f, 0.0f, 1.0f,
             // Back face
             0.0f, 0.0f, -1.0f,
             0.0f, 0.0f, -1.0f,
             0.0f, 0.0f, -1.0f,
             0.0f, 0.0f, -1.0f,
             // Top face
             0.0f, 1.0f, 0.0f,
             0.0f, 1.0f, 0.0f,
             0.0f, 1.0f, 0.0f,
             0.0f, 1.0f, 0.0f,
             // Bottom face
             0.0f, -1.0f, 0.0f,
             0.0f, -1.0f, 0.0f,
             0.0f, -1.0f, 0.0f,
             0.0f, -1.0f, 0.0f,
             // Right face
             1.0f, 0.0f, 0.0f,
             1.0f, 0.0f, 0.0f,
             1.0f, 0.0f, 0.0f,
             1.0f, 0.0f, 0.0f,
             // Left face
             -1.0f, 0.0f, 0.0f,
             -1.0f, 0.0f, 0.0f,
             -1.0f, 0.0f, 0.0f,
             -1.0f, 0.0f, 0.0f},
         // Indices: define two triangles per face
         {
             // Front face
             0, 1, 2, 2, 3, 0,
             // Back face
             4, 5, 6, 6, 7, 4,
             // Top face
             8, 9, 10, 10, 11, 8,
             // Bottom face
             14, 13, 12, 12, 15, 14,
             // Right face
             16, 17, 18, 18, 19, 16,
             // Left face
             20, 21, 22, 22, 23, 20},
         "#E5E4E2"}};

    std::vector<SQLPointCloud> exampleSQLPointClouds = {
        {
            {
                -0.5f,
                -0.5f,
                0.5f,
                0.5f,
                -0.5f,
                0.5f,
                0.5f,
                0.5f,
                0.5f,
                -0.5f,
                0.5f,
                0.5f,
                0.5f,
                -0.5f,
                -0.5f,
                -0.5f,
                -0.5f,
                -0.5f,
                -0.5f,
                0.5f,
                -0.5f,
                0.5f,
                0.5f,
                -0.5f,
            }, // Vertices
            {
                0.0f,
                0.0f,
                1.0f,
                0.0f,
                0.0f,
                1.0f,
                0.0f,
                0.0f,
                1.0f,
                0.0f,
                0.0f,
                1.0f,
                0.0f,
                0.0f,
                1.0f,
                0.0f,
                0.0f,
                1.0f,
                0.0f,
                0.0f,
                1.0f,
                0.0f,
                0.0f,
                1.0f,
            }, // Normals
            {
                0.0f,
                0.0f,
                1.0f,
                0.0f,
                0.0f,
                1.0f,
                0.0f,
                0.0f,
                1.0f,
                0.0f,
                0.0f,
                1.0f,
                0.0f,
                0.0f,
                1.0f,
                0.0f,
                0.0f,
                1.0f,
                0.0f,
                0.0f,
                1.0f,
                0.0f,
                0.0f,
                1.0f,
            } // Colors
        }};

    insertSQLPolylines(db, exampleSQLPolylines, true);

    insertSQLMeshes(db, exampleSQLMeshes, true);

    insertSQLPointClouds(db, exampleSQLPointClouds, true);

    sqlite3_close(db);
}

inline void insertSQLPolylinesClipper(std::vector<SQLPolyline>& SQLPolylines, std::vector<Clipper2Lib::Paths64> &vector_of_paths, std::string color = "#098FCC",  float thickness = 3.0f, double scale = 1, double z = 0){

        for(auto& paths : vector_of_paths){
            for (int i = 0; i < paths.size(); i++)
            {
                SQLPolyline polyline;
                polyline.color = color;
                polyline.thickness = thickness;
                polyline.vertices.reserve(paths[i].size()*3);

                
                for (int j = 0; j < paths[i].size(); j++){
                    polyline.vertices.emplace_back( (float)paths[i][j].x / scale );
                    polyline.vertices.emplace_back( (float)paths[i][j].y / scale );
                    polyline.vertices.emplace_back( z );
                }
                    polyline.vertices.emplace_back( (float)paths[i][0].x / scale );
                    polyline.vertices.emplace_back( (float)paths[i][0].y / scale );
                    polyline.vertices.emplace_back( z );

                SQLPolylines.emplace_back(polyline);
            }
        }
}

inline void write(
        std::vector<Clipper2Lib::Paths64> &pattern_glm,
        std::vector<Clipper2Lib::Paths64> &path_glm,
        std::vector<Clipper2Lib::Paths64> &nfp_glm,
        std::vector<Clipper2Lib::Paths64> &results,
        double scale = 1,
        std::string path = "C:/brg/2_code/database_viewer/cmake/src/viewer/database/database_viewer.db"
)
{
    sqlite3 *db;
    if (sqlite3_open(path.c_str(), &db) == SQLITE_OK)
    {
        // Enable WAL mode
        sqlite3_exec(db, "PRAGMA journal_mode=WAL;", 0, 0, 0);

        // Ensure the tables exist and are properly structured.
        // Continue with the rest of your database operations...
    }
    else
    {
        std::cerr << "Can't open database: " << sqlite3_errmsg(db) << std::endl;
        return;
    }

    // Ensure the tables exist and are properly structured.

    execSql(db, "CREATE TABLE IF NOT EXISTS SQLPolylines (ID INTEGER PRIMARY KEY AUTOINCREMENT, Data TEXT NOT NULL, Color TEXT NOT NULL, Thickness REAL NOT NULL);");
    execSql(db, "CREATE TABLE IF NOT EXISTS SQLMeshes (ID INTEGER PRIMARY KEY AUTOINCREMENT, Data TEXT NOT NULL, Color TEXT NOT NULL);");
    execSql(db, "CREATE TABLE IF NOT EXISTS SQLPointClouds (ID INTEGER PRIMARY KEY AUTOINCREMENT, Data TEXT NOT NULL);");

    // Example data for polylines, meshes, and point clouds
    std::vector<SQLPolyline> SQLPolylines;
    insertSQLPolylinesClipper(SQLPolylines, pattern_glm, "#FF0000", 2.0f, scale, 0.01);
    insertSQLPolylinesClipper(SQLPolylines, path_glm, "#FF00FF", 2.0f, scale, 0.01);
    insertSQLPolylinesClipper(SQLPolylines, nfp_glm, "#000000", 2.0f, scale, 0.00);
    insertSQLPolylinesClipper(SQLPolylines, results, "#000000", 4.0f, scale, 0.02);
    insertSQLPolylines(db, SQLPolylines, true);


    sqlite3_close(db);
}