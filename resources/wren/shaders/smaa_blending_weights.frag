#version 330 core

precision highp float;

#define SMAASampleLevelZeroOffset(tex, coord, offset) texture(tex, coord + float(offset) * RESOLUTION)
#define SMAASampleLevelZero(tex, coord) textureLod(tex, coord, 0.0)
#define SMAA_AREATEX_SELECT(sample) sample.rg

#define RESOLUTION (1.0 / viewportSize)
#define mad(a, b, c) (a * b + c)

#define SMAA_MAX_SEARCH_STEPS 16
#define SMAA_AREATEX_MAX_DISTANCE 16
#define SMAA_AREATEX_MAX_DISTANCE_DIAG 20
#define SMAA_MAX_SEARCH_STEPS_DIAG 8
#define SMAA_AREATEX_PIXEL_SIZE (1.0 / vec2(160.0, 560.0))
#define SMAA_AREATEX_SUBTEX_SIZE (1.0 / 7.0)
#define SMAA_CORNER_ROUNDING 25
#define SMAA_CORNER_ROUNDING_NORM (float(SMAA_CORNER_ROUNDING) / 100.0)

uniform sampler2D inputTextures[3];
uniform vec2 viewportSize;

in vec2 texUv;
in vec4 texUvOffsets[3];
in vec2 vPixcoord;

layout(location = 0) out vec4 result;

/*
 * Conditional move:
 */
void SMAAMovc(bvec2 cond, inout vec2 variable, vec2 value) {
  if (cond.x)
    variable.x = value.x;
  if (cond.y)
    variable.y = value.y;
}

void SMAAMovc(bvec4 cond, inout vec4 variable, vec4 value) {
  SMAAMovc(cond.xy, variable.xy, value.xy);
  SMAAMovc(cond.zw, variable.zw, value.zw);
}

//-----------------------------------------------------------------------------
// Corner Detection Functions

void SMAADetectHorizontalCornerPattern(sampler2D edgesTex, inout vec2 weights, vec4 texcoord, vec2 d) {
  vec2 leftRight = step(d.xy, d.yx);
  vec2 rounding = (1.0 - SMAA_CORNER_ROUNDING_NORM) * leftRight;

  rounding /= leftRight.x + leftRight.y;  // Reduce blending for pixels in the center of a line.

  vec2 factor = vec2(1.0, 1.0);
  factor.x -= rounding.x * SMAASampleLevelZeroOffset(edgesTex, texcoord.xy, ivec2(0, 1)).r;
  factor.x -= rounding.y * SMAASampleLevelZeroOffset(edgesTex, texcoord.zw, ivec2(1, 1)).r;
  factor.y -= rounding.x * SMAASampleLevelZeroOffset(edgesTex, texcoord.xy, ivec2(0, -2)).r;
  factor.y -= rounding.y * SMAASampleLevelZeroOffset(edgesTex, texcoord.zw, ivec2(1, -2)).r;

  weights *= clamp(factor, 0.0, 1.0);
}

void SMAADetectVerticalCornerPattern(sampler2D edgesTex, inout vec2 weights, vec4 texcoord, vec2 d) {
  vec2 leftRight = step(d.xy, d.yx);
  vec2 rounding = (1.0 - SMAA_CORNER_ROUNDING_NORM) * leftRight;

  rounding /= leftRight.x + leftRight.y;

  vec2 factor = vec2(1.0, 1.0);
  factor.x -= rounding.x * SMAASampleLevelZeroOffset(edgesTex, texcoord.xy, ivec2(1, 0)).g;
  factor.x -= rounding.y * SMAASampleLevelZeroOffset(edgesTex, texcoord.zw, ivec2(1, 1)).g;
  factor.y -= rounding.x * SMAASampleLevelZeroOffset(edgesTex, texcoord.xy, ivec2(-2, 0)).g;
  factor.y -= rounding.y * SMAASampleLevelZeroOffset(edgesTex, texcoord.zw, ivec2(-2, 1)).g;

  weights *= clamp(factor, 0.0, 1.0);
}

//-----------------------------------------------------------------------------
// Diagonal Search Functions

/**
 * Allows to decode two binary values from a bilinear-filtered access.
 */
vec2 SMAADecodeDiagBilinearAccess(vec2 e) {
  // Bilinear access for fetching 'e' have a 0.25 offset, and we are
  // interested in the R and G edges:
  //
  // +---G---+-------+
  // |   x o R   x   |
  // +-------+-------+
  //
  // Then, if one of these edge is enabled:
  //   Red:   (0.75 * X + 0.25 * 1) => 0.25 or 1.0
  //   Green: (0.75 * 1 + 0.25 * X) => 0.75 or 1.0
  //
  // This function will unpack the values (mad + mul + round):
  // wolframalpha.com: round(x * abs(5 * x - 5 * 0.75)) plot 0 to 1
  e.r = e.r * abs(5.0 * e.r - 5.0 * 0.75);
  return round(e);
}

vec4 SMAADecodeDiagBilinearAccess(vec4 e) {
  e.rb = e.rb * abs(5.0 * e.rb - 5.0 * 0.75);
  return round(e);
}

/**
 * These functions allows to perform diagonal pattern searches.
 */
vec2 SMAASearchDiag1(sampler2D edgesTex, vec2 texcoord, vec2 dir, out vec2 e) {
  vec4 coord = vec4(texcoord, -1.0, 1.0);
  vec3 t = vec3(RESOLUTION.xy, 1.0);
  while (coord.z < float(SMAA_MAX_SEARCH_STEPS_DIAG - 1) && coord.w > 0.9) {
    coord.xyz = mad(t, vec3(dir, 1.0), coord.xyz);
    e = SMAASampleLevelZero(edgesTex, coord.xy).rg;
    coord.w = dot(e, vec2(0.5, 0.5));
  }
  return coord.zw;
}

vec2 SMAASearchDiag2(sampler2D edgesTex, vec2 texcoord, vec2 dir, out vec2 e) {
  vec4 coord = vec4(texcoord, -1.0, 1.0);
  coord.x += 0.25 * RESOLUTION.x;  // See @SearchDiag2Optimization
  vec3 t = vec3(RESOLUTION.xy, 1.0);
  while (coord.z < float(SMAA_MAX_SEARCH_STEPS_DIAG - 1) && coord.w > 0.9) {
    coord.xyz = mad(t, vec3(dir, 1.0), coord.xyz);

    // @SearchDiag2Optimization
    // Fetch both edges at once using bilinear filtering:
    e = SMAASampleLevelZero(edgesTex, coord.xy).rg;
    e = SMAADecodeDiagBilinearAccess(e);

    // Non-optimized version:
    // e.g = SMAASampleLevelZero(edgesTex, coord.xy).g;
    // e.r = SMAASampleLevelZeroOffset(edgesTex, coord.xy, ivec2(1, 0)).r;

    coord.w = dot(e, vec2(0.5, 0.5));
  }
  return coord.zw;
}

/**
 * Similar to SMAAArea, this calculates the area corresponding to a certain
 * diagonal distance and crossing edges 'e'.
 */
vec2 SMAAAreaDiag(sampler2D areaTex, vec2 dist, vec2 e, int offset) {
  vec2 texcoord = mad(vec2(SMAA_AREATEX_MAX_DISTANCE_DIAG, SMAA_AREATEX_MAX_DISTANCE_DIAG), e, dist);

  // We do a scale and bias for mapping to texel space:
  texcoord = mad(SMAA_AREATEX_PIXEL_SIZE, texcoord, 0.5 * SMAA_AREATEX_PIXEL_SIZE);

  // Diagonal areas are on the second half of the texture:
  texcoord.x += 0.5;

  // Move to proper place, according to the subpixel offset:
  texcoord.y += SMAA_AREATEX_SUBTEX_SIZE * float(offset);

  // Do it!
  return SMAA_AREATEX_SELECT(SMAASampleLevelZero(areaTex, texcoord));
}

/**
 * This searches for diagonal patterns and returns the corresponding weights.
 */
vec2 SMAACalculateDiagWeights(sampler2D edgesTex, sampler2D areaTex, vec2 texcoord, vec2 e, ivec4 subsampleIndices) {
  vec2 weights = vec2(0.0, 0.0);

  // Search for the line ends:
  vec4 d;
  vec2 end;
  if (e.r > 0.0) {
    d.xz = SMAASearchDiag1(edgesTex, texcoord, vec2(-1.0, 1.0), end);
    d.x += float(end.y > 0.9);
  } else
    d.xz = vec2(0.0, 0.0);
  d.yw = SMAASearchDiag1(edgesTex, texcoord, vec2(1.0, -1.0), end);

  if (d.x + d.y > 2.0) {  // d.x + d.y + 1 > 3
    // Fetch the crossing edges:
    vec4 coords = mad(vec4(-d.x + 0.25, d.x, d.y, -d.y - 0.25), RESOLUTION.xyxy, texcoord.xyxy);
    vec4 c;
    c.xy = SMAASampleLevelZeroOffset(edgesTex, coords.xy, ivec2(-1, 0)).rg;
    c.zw = SMAASampleLevelZeroOffset(edgesTex, coords.zw, ivec2(1, 0)).rg;
    c.yxwz = SMAADecodeDiagBilinearAccess(c.xyzw);

    // Non-optimized version:
    // vec4 coords = mad(vec4(-d.x, d.x, d.y, -d.y), RESOLUTION.xyxy, texcoord.xyxy);
    // vec4 c;
    // c.x = SMAASampleLevelZeroOffset(edgesTex, coords.xy, ivec2(-1,  0)).g;
    // c.y = SMAASampleLevelZeroOffset(edgesTex, coords.xy, ivec2( 0,  0)).r;
    // c.z = SMAASampleLevelZeroOffset(edgesTex, coords.zw, ivec2( 1,  0)).g;
    // c.w = SMAASampleLevelZeroOffset(edgesTex, coords.zw, ivec2( 1, -1)).r;

    // Merge crossing edges at each side into a single value:
    vec2 cc = mad(vec2(2.0, 2.0), c.xz, c.yw);

    // Remove the crossing edge if we didn't found the end of the line:
    SMAAMovc(bvec2(step(0.9, d.zw)), cc, vec2(0.0, 0.0));

    // Fetch the areas for this line:
    weights += SMAAAreaDiag(areaTex, d.xy, cc, subsampleIndices.z);
  }

  // Search for the line ends:
  d.xz = SMAASearchDiag2(edgesTex, texcoord, vec2(-1.0, -1.0), end);
  if (SMAASampleLevelZeroOffset(edgesTex, texcoord, ivec2(1, 0)).r > 0.0) {
    d.yw = SMAASearchDiag2(edgesTex, texcoord, vec2(1.0, 1.0), end);
    d.y += float(end.y > 0.9);
  } else
    d.yw = vec2(0.0, 0.0);

  if (d.x + d.y > 2.0) {  // d.x + d.y + 1 > 3
    // Fetch the crossing edges:
    vec4 coords = mad(vec4(-d.x, -d.x, d.y, d.y), RESOLUTION.xyxy, texcoord.xyxy);
    vec4 c;
    c.x = SMAASampleLevelZeroOffset(edgesTex, coords.xy, ivec2(-1, 0)).g;
    c.y = SMAASampleLevelZeroOffset(edgesTex, coords.xy, ivec2(0, -1)).r;
    c.zw = SMAASampleLevelZeroOffset(edgesTex, coords.zw, ivec2(1, 0)).gr;
    vec2 cc = mad(vec2(2.0, 2.0), c.xz, c.yw);

    // Remove the crossing edge if we didn't found the end of the line:
    SMAAMovc(bvec2(step(0.9, d.zw)), cc, vec2(0.0, 0.0));

    // Fetch the areas for this line:
    weights += SMAAAreaDiag(areaTex, d.xy, cc, subsampleIndices.w).gr;
  }

  return weights;
}

float SMAASearchLength(sampler2D searchTex, vec2 e, float bias, float scale) {
  // Not required if searchTex accesses are set to point:
  // vec2 SEARCH_TEX_PIXEL_SIZE = 1.0 / vec2(66.0, 33.0);
  // e = vec2(bias, 0.0) + 0.5 * SEARCH_TEX_PIXEL_SIZE +
  //     e * vec2(scale, 1.0) * vec2(64.0, 32.0) * SEARCH_TEX_PIXEL_SIZE;
  e.r = bias + e.r * scale;
  return 255.0 * texture(searchTex, e).r;
}

float SMAASearchXLeft(sampler2D edgesTex, sampler2D searchTex, vec2 texcoord, float end) {
  /**
   * @PSEUDO_GATHER4
   * This texcoord has been offset by (-0.25, -0.125) in the vertex shader to
   * sample between edge, thus fetching four edges in a row.
   * Sampling with different offsets in each direction allows to disambiguate
   * which edges are active from the four fetched ones.
   */
  vec2 e = vec2(0.0, 1.0);

  for (int i = 0; i < SMAA_MAX_SEARCH_STEPS; i++) {  // WebGL port note: Changed while to for
    e = texture(edgesTex, texcoord).rg;
    texcoord -= vec2(2.0, 0.0) * RESOLUTION;
    if (!(texcoord.x > end && e.g > 0.8281 && e.r == 0.0))
      break;
  }

  // We correct the previous (-0.25, -0.125) offset we applied:
  texcoord.x += 0.25 * RESOLUTION.x;

  // The searches are bias by 1, so adjust the coords accordingly:
  texcoord.x += RESOLUTION.x;

  // Disambiguate the length added by the last step:
  texcoord.x += 2.0 * RESOLUTION.x;  // Undo last step
  texcoord.x -= RESOLUTION.x * SMAASearchLength(searchTex, e, 0.0, 0.5);

  return texcoord.x;
}

float SMAASearchXRight(sampler2D edgesTex, sampler2D searchTex, vec2 texcoord, float end) {
  vec2 e = vec2(0.0, 1.0);

  for (int i = 0; i < SMAA_MAX_SEARCH_STEPS; i++) {  // WebGL port note: Changed while to for
    e = texture(edgesTex, texcoord, 0.0).rg;
    texcoord += vec2(2.0, 0.0) * RESOLUTION;
    if (!(texcoord.x < end && e.g > 0.8281 && e.r == 0.0))
      break;
  }

  texcoord.x -= 0.25 * RESOLUTION.x;
  texcoord.x -= RESOLUTION.x;
  texcoord.x -= 2.0 * RESOLUTION.x;
  texcoord.x += RESOLUTION.x * SMAASearchLength(searchTex, e, 0.5, 0.5);

  return texcoord.x;
}

float SMAASearchYUp(sampler2D edgesTex, sampler2D searchTex, vec2 texcoord, float end) {
  vec2 e = vec2(1.0, 0.0);

  for (int i = 0; i < SMAA_MAX_SEARCH_STEPS; i++) {  // WebGL port note: Changed while to for
    e = texture(edgesTex, texcoord, 0.0).rg;
    texcoord += vec2(0.0, 2.0) * RESOLUTION;  // WebGL port note: Changed sign
    if (!(texcoord.y > end && e.r > 0.8281 && e.g == 0.0))
      break;
  }

  texcoord.y -= 0.25 * RESOLUTION.y;                                         // WebGL port note: Changed sign
  texcoord.y -= RESOLUTION.y;                                                // WebGL port note: Changed sign
  texcoord.y -= 2.0 * RESOLUTION.y;                                          // WebGL port note: Changed sign
  texcoord.y += RESOLUTION.y * SMAASearchLength(searchTex, e.gr, 0.0, 0.5);  // WebGL port note: Changed sign

  return texcoord.y;
}

float SMAASearchYDown(sampler2D edgesTex, sampler2D searchTex, vec2 texcoord, float end) {
  vec2 e = vec2(1.0, 0.0);

  for (int i = 0; i < SMAA_MAX_SEARCH_STEPS; i++) {
    e = texture(edgesTex, texcoord, 0.0).rg;
    texcoord -= vec2(0.0, 2.0) * RESOLUTION;
    if (!(texcoord.y < end && e.r > 0.8281 && e.g == 0.0))
      break;
  }

  texcoord.y += 0.25 * RESOLUTION.y;                                         // WebGL port note: Changed sign
  texcoord.y += RESOLUTION.y;                                                // WebGL port note: Changed sign
  texcoord.y += 2.0 * RESOLUTION.y;                                          // WebGL port note: Changed sign
  texcoord.y -= RESOLUTION.y * SMAASearchLength(searchTex, e.gr, 0.5, 0.5);  // WebGL port note: Changed sign

  return texcoord.y;
}

vec2 SMAAArea(sampler2D areaTex, vec2 dist, float e1, float e2, float offset) {
  // Rounding prevents precision errors of bilinear filtering:
  vec2 texcoord = float(SMAA_AREATEX_MAX_DISTANCE) * round(4.0 * vec2(e1, e2)) + dist;

  // We do a scale and bias for mapping to texel space:
  texcoord = SMAA_AREATEX_PIXEL_SIZE * texcoord + (0.5 * SMAA_AREATEX_PIXEL_SIZE);

  // Move to proper place, according to the subpixel offset:
  texcoord.y += SMAA_AREATEX_SUBTEX_SIZE * offset;

  return texture(areaTex, texcoord, 0.0).rg;
}

vec4 SMAABlendingWeightCalculationPS(vec2 texcoord, vec2 pixcoord, vec4 offset[3], sampler2D edgesTex, sampler2D areaTex,
                                     sampler2D searchTex, ivec4 subsampleIndices) {
  vec4 weights = vec4(0.0, 0.0, 0.0, 0.0);

  vec2 e = texture(edgesTex, texcoord).rg;

  if (e.g > 0.0) {  // Edge at north

    // Diagonals have both north and west edges, so searching for them in
    // one of the boundaries is enough.
    weights.rg = SMAACalculateDiagWeights(edgesTex, areaTex, texcoord, e, subsampleIndices);

    // We give priority to diagonals, so if we find a diagonal we skip
    // horizontal/vertical processing.
    if (weights.r == -weights.g) {  // weights.r + weights.g == 0.0

      vec2 d;

      // Find the distance to the left:
      vec2 coords;
      coords.x = SMAASearchXLeft(edgesTex, searchTex, offset[0].xy, offset[2].x);
      coords.y = offset[1].y;  // offset[1].y = texcoord.y - 0.25 * RESOLUTION.y (@CROSSING_OFFSET)
      d.x = coords.x;

      // Now fetch the left crossing edges, two at a time using bilinear
      // filtering. Sampling at -0.25 (see @CROSSING_OFFSET) enables to
      // discern what value each edge has:
      float e1 = texture(edgesTex, coords, 0.0).r;

      // Find the distance to the right:
      coords.x = SMAASearchXRight(edgesTex, searchTex, offset[0].zw, offset[2].y);
      d.y = coords.x;

      // We want the distances to be in pixel units (doing this here allow to
      // better interleave arithmetic and memory accesses):
      d = d / RESOLUTION.x - pixcoord.x;

      // SMAAArea below needs a sqrt, as the areas texture is compressed
      // quadratically:
      vec2 sqrt_d = sqrt(abs(d));

      // Fetch the right crossing edges:
      coords.y -= 1.0 * RESOLUTION.y;  // WebGL port note: Added
      float e2 = SMAASampleLevelZeroOffset(edgesTex, coords, ivec2(1, 0)).r;

      // Ok, we know how this pattern looks like, now it is time for getting
      // the actual area:
      weights.rg = SMAAArea(areaTex, sqrt_d, e1, e2, float(subsampleIndices.y));

      // fix corners
      coords.y = texcoord.y;
      SMAADetectHorizontalCornerPattern(edgesTex, weights.rg, coords.xyxy, d);

    } else
      e.r = 0.0;  // Skip vertical processing.
  }

  if (e.r > 0.0) {  // Edge at west
    vec2 d;

    // Find the distance to the top:
    vec2 coords;

    coords.y = SMAASearchYUp(edgesTex, searchTex, offset[1].xy, offset[2].z);
    coords.x = offset[0].x;  // offset[1].x = texcoord.x - 0.25 * RESOLUTION.x;
    d.x = coords.y;

    // Fetch the top crossing edges:
    float e1 = texture(edgesTex, coords, 0.0).g;

    // Find the distance to the bottom:
    coords.y = SMAASearchYDown(edgesTex, searchTex, offset[1].zw, offset[2].w);
    d.y = coords.y;

    // We want the distances to be in pixel units:
    d = d / RESOLUTION.y - pixcoord.y;

    // SMAAArea below needs a sqrt, as the areas texture is compressed
    // quadratically:
    vec2 sqrt_d = sqrt(abs(d));

    // Fetch the bottom crossing edges:
    coords.y -= 1.0 * RESOLUTION.y;  // WebGL port note: Added
    float e2 = SMAASampleLevelZeroOffset(edgesTex, coords, ivec2(0, 1)).g;

    // Get the area for this direction:
    weights.ba = SMAAArea(areaTex, sqrt_d, e1, e2, float(subsampleIndices.x));

    // Fix corners
    coords.x = texcoord.x;
    SMAADetectVerticalCornerPattern(edgesTex, weights.ba, coords.xyxy, d);
  }

  return vec4(weights);
}

void main() {
  result = SMAABlendingWeightCalculationPS(texUv, vPixcoord, texUvOffsets, inputTextures[0], inputTextures[1], inputTextures[2],
                                           ivec4(0.0));
}
