/**
 * @brief 2D position structure - inherited from 2D point structure
 *
 */
class Pose2D : public Point2D{
    public:
    double theta{0};

    /**
     * @brief Construct a new Pose 2D object
     *
     * @param _x x value
     * @param _y y value
     * @param _heading angle
     */
    Pose2D(const double& _x, const double& _y, const double& _heading = 0);

    /**
     * @brief Construct a new Pose 2D object
     *
     * @param coord x & y value
     * @param _heading angle
     */
    Pose2D(const Point2D& coord, const double& _heading = 0);

    /**
     * @brief Destruct a Point 2D object
     *
     */
    virtual ~Pose2D() = default;

    /**
     * @brief Pose 2D operators
     *
     * @param rhs right hand side of the operation
     */
    Pose2D& operator=(const Pose2D& rhs);
    bool operator==(const Pose2D& rhs) const;
    bool operator!=(const Pose2D& rhs) const;
    Pose2D operator+(const Pose2D& rhs) const;
    Pose2D& operator+=(const Pose2D& rhs);
    Pose2D operator-(const Pose2D& rhs) const;
    Pose2D& operator-=(const Pose2D& rhs);

    /**
     * @brief Pose 2D operators (with points)
     *
     * @param rhs right hand side of the operation
     */
    Pose2D operator+(const Point2D& rhs) const;
    Pose2D& operator+=(const Point2D& rhs);
    Pose2D operator-(const Point2D& rhs) const;
    Pose2D& operator-=(const Point2D& rhs);

    /**
     * @brief Pose 2D scalars
     *
     * @param rhs right hand side of the operation
     */
    Pose2D operator*(const double& rhs) const;
    Pose2D& operator*=(const double& rhs);
    Pose2D operator/(const double& rhs) const;
    Pose2D& operator/=(const double& rhs);

    /**
     * @brief Angle to targeet point / pose
     *
     * @param target target point / pose
     * @return Angle between the 2 points
     */
    double angleTo(const Point2D& target) const override;
    double angleTo(const Pose2D& target) const;

    /**
     * @brief Distance to target
     *
     * @param target target point / pose
     * @return distance between the 2 points
     */
    double distanceTo(const Pose2D& target) const;

    /**
     * @brief Calculate the point along a given heading that is closest (perpendicular) to a target point.
     *
     * @param target target point
     */
    Point2D closest(const Point2D& target) const;

    /**
     * @brief Normalizes the pose
     *
     * @return the normalized pose
     */
    Pose2D normalize() const;

    /**
     * @brief Converts pose to point
     *
     */
    Point2D toPoint() const;
};  