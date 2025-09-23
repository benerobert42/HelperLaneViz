#include "MathUtilities.h"

constexpr simd_float3 add(const simd_float3& a, const simd_float3& b)
{
    return { a.x + b.x, a.y + b.y, a.z + b.z };
}

constexpr simd_float4x4 makeIdentity()
{
    using simd::float4;
    return (simd_float4x4){ (float4){ 1.f, 0.f, 0.f, 0.f },
                            (float4){ 0.f, 1.f, 0.f, 0.f },
                            (float4){ 0.f, 0.f, 1.f, 0.f },
                            (float4){ 0.f, 0.f, 0.f, 1.f } };
}

simd_float4x4 makePerspective(float fovRadians, float aspect, float znear, float zfar)
{
    using simd::float4;
    float ys = 1.f / tanf(fovRadians * 0.5f);
    float xs = ys / aspect;
    float zs = zfar / ( znear - zfar );
    return simd_matrix_from_rows((float4){ xs, 0.0f, 0.0f, 0.0f },
                                 (float4){ 0.0f, ys, 0.0f, 0.0f },
                                 (float4){ 0.0f, 0.0f, zs, znear * zs },
                                 (float4){ 0, 0, -1, 0 });
}

matrix_float4x4 makeOrthoLhs(float left, float right, float bottom, float top, float nearZ, float farZ) {
    using simd::float4;
    return simd_matrix_from_rows((float4){2 / (right - left), 0, 0, (left + right) / (left - right)},
                                 (float4){0, 2 / (top - bottom), 0, (top + bottom) / (bottom - top)},
                                 (float4){0, 0, 1 / (farZ - nearZ), nearZ / (nearZ - farZ)},
                                 (float4){0, 0, 0, 1 });
}

matrix_float4x4 makeOrthoRhs(float left, float right, float bottom, float top, float nearZ, float farZ) {
    using simd::float4;
    return simd_matrix_from_rows((float4){2 / (right - left), 0, 0, (left + right) / (left - right)},
                                 (float4){0, 2 / (top - bottom), 0, (top + bottom) / (bottom - top)},
                                 (float4){0, 0, -1 / (farZ - nearZ), nearZ / (nearZ - farZ)},
                                 (float4){0, 0, 0, 1});
}

simd_float4x4 makeXRotate(float angleRadians)
{
    using simd::float4;
    const float a = angleRadians;
    return simd_matrix_from_rows((float4){ 1.0f, 0.0f, 0.0f, 0.0f },
                                 (float4){ 0.0f, cosf( a ), sinf( a ), 0.0f },
                                 (float4){ 0.0f, -sinf( a ), cosf( a ), 0.0f },
                                 (float4){ 0.0f, 0.0f, 0.0f, 1.0f });
}

simd_float4x4 makeYRotate(float angleRadians)
{
    using simd::float4;
    const float a = angleRadians;
    return simd_matrix_from_rows((float4){ cosf( a ), 0.0f, sinf( a ), 0.0f },
                                 (float4){ 0.0f, 1.0f, 0.0f, 0.0f },
                                 (float4){ -sinf( a ), 0.0f, cosf( a ), 0.0f },
                                 (float4){ 0.0f, 0.0f, 0.0f, 1.0f });
}

simd_float4x4 makeZRotate(float angleRadians)
{
    using simd::float4;
    const float a = angleRadians;
    return simd_matrix_from_rows((float4){ cosf( a ), sinf( a ), 0.0f, 0.0f },
                                 (float4){ -sinf( a ), cosf( a ), 0.0f, 0.0f },
                                 (float4){ 0.0f, 0.0f, 1.0f, 0.0f },
                                 (float4){ 0.0f, 0.0f, 0.0f, 1.0f });
}

simd_float4x4 makeRotate(simd_float3 rotation) {
    simd_float4x4 xRotate = makeXRotate(rotation.x);
    simd_float4x4 yRotate = makeYRotate(rotation.y);
    simd_float4x4 zRotate = makeZRotate(rotation.z);
    return simd_mul(xRotate, simd_mul(yRotate, zRotate));

}

simd_float4x4 makeTranslate(const simd_float3& v)
{
    using simd::float4;
    const float4 col0 = { 1.0f, 0.0f, 0.0f, 0.0f };
    const float4 col1 = { 0.0f, 1.0f, 0.0f, 0.0f };
    const float4 col2 = { 0.0f, 0.0f, 1.0f, 0.0f };
    const float4 col3 = { v.x, v.y, v.z, 1.0f };
    return simd_matrix( col0, col1, col2, col3 );
}

simd_float4x4 makeScale(const simd_float3& v)
{
    using simd::float4;
    return simd_matrix((float4){ v.x, 0, 0, 0 },
                       (float4){ 0, v.y, 0, 0 },
                       (float4){ 0, 0, v.z, 0 },
                       (float4){ 0, 0, 0, 1.0 });
}

simd_float4x4 discardTranslation(const simd_float4x4& m)
{
    return simd_matrix( m.columns[0], m.columns[1], m.columns[2], simd_make_float4(0.0, 0.0, 0.0, 1.0) );
}

simd_float3x3 normalTransform(const simd_float4x4& m)
{
    simd_float3x3 upper3x3ModelMatrix = simd_matrix(m.columns[0].xyz, m.columns[1].xyz, m.columns[2].xyz);
    upper3x3ModelMatrix = simd_inverse(upper3x3ModelMatrix);
    return simd_transpose(upper3x3ModelMatrix);
}

simd_float4x4 createLookAtLhs(simd_float3 eye,
                              simd_float3 target,
                              simd_float3 up) {
    vector_float3 z = vector_normalize(target - eye);
    vector_float3 x = vector_normalize(vector_cross(up, z));
    vector_float3 y = vector_cross(z, x);
    vector_float3 t = simd_make_float3(-simd_dot(x, eye), -simd_dot(y, eye), -simd_dot(z, eye));

    using simd::float4;
    return simd_matrix_from_rows((float4){x.x, x.y, x.z, t.x},
                                 (float4){y.x, y.y, y.z, t.y},
                                 (float4){z.x, z.y, z.z, t.z},
                                 (float4){0, 0, 0, 1});
}

matrix_float4x4 createLookAtRhs(vector_float3 eye,
                                vector_float3 target,
                                vector_float3 up) {
    vector_float3 z = vector_normalize(eye - target);
    vector_float3 x = vector_normalize(vector_cross(up, z));
    vector_float3 y = vector_cross(z, x);
    vector_float3 t = simd_make_float3(-simd_dot(x, eye), -simd_dot(y, eye), -simd_dot(z, eye));

    using simd::float4;
    return simd_matrix_from_rows((float4){x.x, x.y, x.z, t.x},
                                 (float4){y.x, y.y, y.z, t.y},
                                 (float4){z.x, z.y, z.z, t.z},
                                 (float4){0, 0, 0, 1});
}

