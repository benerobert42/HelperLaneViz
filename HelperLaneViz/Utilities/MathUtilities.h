#pragma once

#include <simd/simd.h>

constexpr simd_float3 add(const simd_float3& a, const simd_float3& b);

constexpr simd_float4x4 makeIdentity();

simd_float4x4 makePerspective(float fovRadians, float aspect, float znear, float zfar);

matrix_float4x4 makeOrthoLhs(float left, float right, float bottom, float top, float nearZ, float farZ);

matrix_float4x4 makeOrthoRhs(float left, float right, float bottom, float top, float nearZ, float farZ);

simd_float4x4 makeXRotate(float angleRadians);

simd_float4x4 makeYRotate(float angleRadians);

simd_float4x4 makeZRotate(float angleRadians);

simd_float4x4 makeRotate(simd_float3 rotation);

simd_float4x4 makeTranslate(const simd_float3& v);

simd_float4x4 makeScale(const simd_float3& v);

simd_float4x4 discardTranslation(const simd_float4x4& m);

simd_float3x3 normalTransform(const simd_float4x4& m);

simd_float4x4 createLookAtLhs(simd_float3 eye,
                              simd_float3 target,
                              simd_float3 up);

simd_float4x4 createLookAtRhs(simd_float3 eye,
                              simd_float3 target,
                              simd_float3 up);
