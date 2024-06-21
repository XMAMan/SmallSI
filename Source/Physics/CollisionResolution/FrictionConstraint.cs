using Physics.CollisionDetection;
using Physics.Math;

namespace Physics.CollisionResolution
{
    internal class FrictionConstraint : IConstraint
    {
        public RigidRectangle B1 { get; }
        public RigidRectangle B2 { get; }
        public Vec2D R1 { get; }
        public Vec2D R2 { get; }
        public float MinImpulse { get; } = 0;
        public float MaxImpulse { get; } = float.MaxValue;

        public Vec2D ForceDirection { get; }
        public float Bias { get; } = 0;
        public float ImpulseMass { get; }
        public float AccumulatedImpulse { get; set; } = 0;
        public FrictionConstraint(Settings settings, CollisionInfo c)
        {
            Vec2D p = ResolutionHelper.GetContactPoint(c);

            this.B1 = c.B1;
            this.B2 = c.B2;
            this.R1 = p - c.B1.Center;
            this.R2 = p - c.B2.Center;

            Vec2D tangent = Vec2D.CrossWithZ(c.Normal, 1.0f);
            float r1crossT = Vec2D.ZValueFromCross(R1, tangent);
            float r2crossT = Vec2D.ZValueFromCross(R2, tangent);

            this.ImpulseMass = 1.0f / (B1.InverseMass + B2.InverseMass +
                r1crossT * r1crossT * B1.InverseInertia +
                r2crossT * r2crossT * B2.InverseInertia);

            this.ForceDirection = tangent;

            float friction = System.Math.Max(c.B1.Friction, c.B2.Friction);
            this.MaxImpulse = settings.Gravity * friction * settings.Dt;
            this.MinImpulse = -MaxImpulse;
        }

    }
}
