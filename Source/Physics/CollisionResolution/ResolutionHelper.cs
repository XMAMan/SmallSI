using Physics.CollisionDetection;
using Physics.Math;

namespace Physics.CollisionResolution
{
    internal static class ResolutionHelper
    {
        public static Vec2D GetContactPoint(CollisionInfo c)
        {
            Vec2D start = c.Start * (c.B2.InverseMass / (c.B1.InverseMass + c.B2.InverseMass));
            Vec2D end = c.End * (c.B1.InverseMass / (c.B1.InverseMass + c.B2.InverseMass));
            return start + end;
        }

        public static Vec2D GetRelativeVelocityBetweenAnchorPoints(RigidRectangle b1, RigidRectangle b2, Vec2D r1, Vec2D r2)
        {
            Vec2D v1 =b1.Velocity + new Vec2D(-b1.AngularVelocity * r1.Y, b1.AngularVelocity * r1.X);
            Vec2D v2 =b2.Velocity + new Vec2D(-b2.AngularVelocity * r2.Y, b2.AngularVelocity * r2.X);
            return v2 - v1;
        }

        public static float Clamp(float a, float low, float high)
        {
            return System.Math.Max(low, System.Math.Min(a, high));
        }
    }
}
