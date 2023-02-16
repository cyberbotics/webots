"""Math utilities."""


def apply_spline_subdivision_to_path(path, subdivision):
    """Apply spline subdivision to the list of points."""
    spline = []
    points = list(path)
    pointsNumber = len(points)
    # extend the points array
    points.append((points[pointsNumber - 1][0] + (points[pointsNumber - 1][0] - points[pointsNumber - 2][0]),
                   points[pointsNumber - 1][1] + (points[pointsNumber - 1][1] - points[pointsNumber - 2][1]),
                   points[pointsNumber - 1][2] + (points[pointsNumber - 1][2] - points[pointsNumber - 2][2])))
    points.append((points[pointsNumber][0] + (points[pointsNumber][0] - points[pointsNumber - 1][0]),
                   points[pointsNumber][1] + (points[pointsNumber][1] - points[pointsNumber - 1][1]),
                   points[pointsNumber][2] + (points[pointsNumber][2] - points[pointsNumber - 1][2])))
    pointMinus1 = (points[0][0] + (points[0][0] - points[1][0]),
                   points[0][1] + (points[0][1] - points[1][1]),
                   points[0][2] + (points[0][2] - points[1][2]))
    pointMinus2 = (pointMinus1[0] + (pointMinus1[0] - points[0][0]),
                   pointMinus1[1] + (pointMinus1[1] - points[0][1]),
                   pointMinus1[2] + (pointMinus1[2] - points[0][2]))
    # adding to the end is equivalent to index -1 and -2
    points.append(pointMinus2)
    points.append(pointMinus1)

    # interpolation
    spline.append(points[0])  # first point
    for i in range(1, pointsNumber):
        # compute the third order coefficients
        coefficients = []
        for j in range(3):
            coefficients.append([
                (-points[i - 2][j] + 3 * points[i - 1][j] - 3 * points[i][j] + points[i + 1][j]) / 6.0,
                (3 * points[i - 2][j] - 6 * points[i - 1][j] + 3 * points[i][j]) / 6.0,
                (-3 * points[i - 2][j] + 3 * points[i][j]) / 6.0,
                (points[i - 2][j] + 4 * points[i - 1][j] + points[i][j]) / 6.0
            ])
        for j in range(subdivision):
            t = float(j + 1) / subdivision
            spline.append(((coefficients[0][2] + t * (coefficients[0][1] + t * coefficients[0][0])) * t + coefficients[0][3],
                           (coefficients[1][2] + t * (coefficients[1][1] + t * coefficients[1][0])) * t + coefficients[1][3],
                           (coefficients[2][2] + t * (coefficients[2][1] + t * coefficients[2][0])) * t + coefficients[2][3]))
    return spline
