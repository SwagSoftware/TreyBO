#include "obj_loader.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unordered_map>

namespace
{
struct corner
{
    int v, vt, vn; // 0-based, -1 = absent
};

struct corner_hash
{
    size_t operator()(const corner &c) const
    {
        return ((size_t)(uint32_t)c.v * 73856093u) ^ ((size_t)(uint32_t)c.vt * 19349663u) ^ ((size_t)(uint32_t)c.vn * 83492791u);
    }
};

struct corner_eq
{
    bool operator()(const corner &a, const corner &b) const { return a.v == b.v && a.vt == b.vt && a.vn == b.vn; }
};

const char *skip_ws(const char *p)
{
    while (*p == ' ' || *p == '\t')
        ++p;
    return p;
}

// OBJ index: 1-based, or negative (relative to the current end).
int fix_index(long i, int count)
{
    if (i > 0)
        return (int)i - 1;
    if (i < 0)
        return count + (int)i;
    return -1;
}

bool read_file(const char *path, std::vector<char> *data)
{
    FILE *f = fopen(path, "rb");
    if (!f)
        return false;
    fseek(f, 0, SEEK_END);
    const long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (size < 0)
    {
        fclose(f);
        return false;
    }
    data->resize((size_t)size + 1);
    const size_t got = fread(data->data(), 1, (size_t)size, f);
    fclose(f);
    (*data)[got] = 0;
    data->resize(got + 1);
    return true;
}
} // namespace

bool obj_load(const char *path, obj_mesh *out, std::string *error)
{
    std::vector<char> data;
    if (!read_file(path, &data))
    {
        if (error)
            *error = std::string("cannot open ") + path;
        return false;
    }

    std::vector<float> uvs, normals;
    std::unordered_map<corner, uint32_t, corner_hash, corner_eq> corner_map;
    std::vector<corner> face;
    std::vector<uint32_t> face_render;
    std::vector<uint8_t> needs_flat; // per render vertex: no normal in the file
    out->positions.clear();
    out->vertices.clear();
    out->indices.clear();
    out->position_indices.clear();
    out->face_count = 0;

    const char *p = data.data();
    while (*p)
    {
        const char *line = skip_ws(p);
        const char *eol = line;
        while (*eol && *eol != '\n' && *eol != '\r')
            ++eol;
        p = eol;
        while (*p == '\n' || *p == '\r')
            ++p;

        if (line[0] == 'v' && (line[1] == ' ' || line[1] == '\t'))
        {
            char *e;
            const float x = strtof(line + 2, &e);
            const float y = strtof(e, &e);
            const float z = strtof(e, &e);
            out->positions.push_back(x);
            out->positions.push_back(y);
            out->positions.push_back(z);
        }
        else if (line[0] == 'v' && line[1] == 't' && (line[2] == ' ' || line[2] == '\t'))
        {
            char *e;
            const float u = strtof(line + 3, &e);
            const float v = strtof(e, &e);
            uvs.push_back(u);
            uvs.push_back(v);
        }
        else if (line[0] == 'v' && line[1] == 'n' && (line[2] == ' ' || line[2] == '\t'))
        {
            char *e;
            const float x = strtof(line + 3, &e);
            const float y = strtof(e, &e);
            const float z = strtof(e, &e);
            normals.push_back(x);
            normals.push_back(y);
            normals.push_back(z);
        }
        else if (line[0] == 'f' && (line[1] == ' ' || line[1] == '\t'))
        {
            const int vcount = (int)(out->positions.size() / 3);
            const int tcount = (int)(uvs.size() / 2);
            const int ncount = (int)(normals.size() / 3);
            face.clear();
            const char *q = line + 2;
            while (q < eol)
            {
                q = skip_ws(q);
                if (q >= eol)
                    break;
                char *e;
                corner c = {-1, -1, -1};
                c.v = fix_index(strtol(q, &e, 10), vcount);
                q = e;
                if (*q == '/')
                {
                    ++q;
                    if (*q != '/')
                    {
                        c.vt = fix_index(strtol(q, &e, 10), tcount);
                        q = e;
                    }
                    if (*q == '/')
                    {
                        ++q;
                        c.vn = fix_index(strtol(q, &e, 10), ncount);
                        q = e;
                    }
                }
                while (q < eol && *q != ' ' && *q != '\t')
                    ++q; // tolerate junk
                if (c.v < 0 || c.v >= vcount)
                {
                    if (error)
                        *error = "face references a missing vertex";
                    return false;
                }
                if (c.vt >= tcount)
                    c.vt = -1;
                if (c.vn >= ncount)
                    c.vn = -1;
                face.push_back(c);
            }
            if (face.size() < 3)
                continue;
            ++out->face_count;

            face_render.clear();
            for (const corner &c : face)
            {
                auto it = corner_map.find(c);
                if (it == corner_map.end())
                {
                    obj_vertex v;
                    memcpy(v.pos, &out->positions[(size_t)c.v * 3], sizeof(v.pos));
                    if (c.vn >= 0)
                        memcpy(v.nrm, &normals[(size_t)c.vn * 3], sizeof(v.nrm));
                    else
                        v.nrm[0] = v.nrm[1] = v.nrm[2] = 0.0f;
                    if (c.vt >= 0)
                        memcpy(v.uv, &uvs[(size_t)c.vt * 2], sizeof(v.uv));
                    else
                        v.uv[0] = v.uv[1] = 0.0f;
                    const uint32_t idx = (uint32_t)out->vertices.size();
                    out->vertices.push_back(v);
                    needs_flat.push_back(c.vn < 0 ? 1 : 0);
                    it = corner_map.emplace(c, idx).first;
                }
                face_render.push_back(it->second);
            }
            // Fan triangulation (faces in these assets are convex; triangles pass through unchanged).
            for (size_t i = 1; i + 1 < face.size(); ++i)
            {
                out->indices.push_back(face_render[0]);
                out->indices.push_back(face_render[i]);
                out->indices.push_back(face_render[i + 1]);
                out->position_indices.push_back((uint32_t)face[0].v);
                out->position_indices.push_back((uint32_t)face[i].v);
                out->position_indices.push_back((uint32_t)face[i + 1].v);
            }
        }
        else if (line[0] == 'o' && (line[1] == ' ' || line[1] == '\t') && out->object_name.empty())
        {
            out->object_name.assign(skip_ws(line + 2), eol);
        }
    }

    // Accumulate area-weighted face normals for corners that had none.
    for (size_t t = 0; t + 2 < out->indices.size(); t += 3)
    {
        obj_vertex *v[3] = {&out->vertices[out->indices[t]], &out->vertices[out->indices[t + 1]], &out->vertices[out->indices[t + 2]]};
        const float e1[3] = {v[1]->pos[0] - v[0]->pos[0], v[1]->pos[1] - v[0]->pos[1], v[1]->pos[2] - v[0]->pos[2]};
        const float e2[3] = {v[2]->pos[0] - v[0]->pos[0], v[2]->pos[1] - v[0]->pos[1], v[2]->pos[2] - v[0]->pos[2]};
        const float n[3] = {e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]};
        for (int k = 0; k < 3; ++k)
        {
            if (needs_flat[out->indices[t + k]])
                for (int c = 0; c < 3; ++c)
                    v[k]->nrm[c] += n[c];
        }
    }
    for (size_t i = 0; i < out->vertices.size(); ++i)
    {
        float *n = out->vertices[i].nrm;
        const float l = sqrtf(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        if (l > 1e-20f)
        {
            n[0] /= l;
            n[1] /= l;
            n[2] /= l;
        }
        else
        {
            n[0] = 0.0f;
            n[1] = 0.0f;
            n[2] = 1.0f;
        }
    }

    out->uv_count = (int)(uvs.size() / 2);
    out->normal_count = (int)(normals.size() / 3);
    if (out->indices.empty())
    {
        if (error)
            *error = "no faces";
        return false;
    }
    return true;
}
