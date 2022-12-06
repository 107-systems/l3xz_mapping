#include <l3xz_mapping/pose.hpp>

Pose::Pose()
{
    x = 0.0;
    y = 0.0;
    z = 0.0;
    yaw = 0.0;
    pitch = 0.0;
    roll = 0.0;
}

std::string Pose::str()
{
    std::string s = "x: " + std::to_string(x) + ", y: " + std::to_string(y) +
                    ", z: " + std::to_string(z) + ", yaw: " + std::to_string(yaw / M_PI * 180.0) +
                    ", pitch: " + std::to_string(pitch / M_PI * 180.0) +
                    ", roll: " + std::to_string(roll / M_PI * 180.0);
    return s;
}

bool Pose::operator==(const Pose &p)
{
    return (x == p.x) && (y = p.y) && (z == p.z) && (yaw == p.yaw) && (pitch == p.pitch) &&
           (roll == p.roll);
}

Pose Pose::operator+(const Pose &p)
{
    Pose out;
    out.x = this->x + p.x;
    out.y = this->y + p.y;
    out.z = this->z + p.z;
    out.yaw = this->yaw + p.yaw;
    out.pitch = this->pitch + p.pitch;
    out.roll = this->roll + p.roll;
    return out;
}

Pose &Pose::operator+=(const Pose &p)
{
    this->x += p.x;
    this->y += p.y;
    this->z += p.z;
    this->yaw += p.yaw;
    this->pitch += p.pitch;
    this->roll += p.roll;
    return *this;
}

double Pose::dist2D(const Pose &p) { return sqrt((p.x - x) * (p.x - x) + (p.y - y) * (p.y - y)); }

double Pose::angle(const Pose &p) { return -atan2(p.y - this->y, p.x - this->x); }

double Pose::dY(const Pose &p) { return sin(angle(p)) * dist2D(p); }

double Pose::dX(const Pose &p) { return cos(angle(p)) * dist2D(p); }

void Pose::rotatePitch(double r)
{
    double yy = 0.0;
    double zz = 0.0;
    yy += static_cast<double>(y * cos(r) - z * sin(r));
    zz += static_cast<double>(y * sin(r) + z * cos(r));
    y = yy;
    z = zz;
}

void Pose::rotateRoll(double r)
{
    double xx = 0.0;
    double zz = 0.0;

    xx += static_cast<double>(x * cos(r) + z * sin(r));
    zz += static_cast<double>(-x * sin(r) + z * cos(r));
    x = xx;
    z = zz;
}

void Pose::rotateYaw(double r)
{
    double xx = 0.0;
    double yy = 0.0;
    xx += static_cast<double>(x * cos(r) - y * sin(r));
    yy += static_cast<double>(x * sin(r) + y * cos(r));
    x = xx;
    y = yy;
}
