using Physics.CollisionDetection;
using Physics.Math;

namespace Physics.CollisionResolution
{
    internal class NormalConstraint : IConstraint
    {
        public RigidRectangle B1 { get; }
        public RigidRectangle B2 { get; }
        public Vec2D R1 { get; } //Lever arm from B1.Center to contact point
        public Vec2D R2 { get; } //Lever arm from B2.Center to contact point
        public float MinImpulse { get; } = 0;
        public float MaxImpulse { get; } = float.MaxValue;

        public Vec2D ForceDirection { get; } //B2 is pressed in this direction (B1 is pressed in the opposite direction)
        public float Bias { get; }
        public float EffectiveMass { get; } //Conversionfactor from the relative contact point velocity to a impulse
        public float AccumulatedImpulse { get; set; } = 0;

        public NormalConstraint(Settings settings, CollisionInfo c)
        {
            Vec2D p = ResolutionHelper.GetContactPoint(c);

            this.B1 = c.B1;
            this.B2 = c.B2;
            this.R1 = p - c.B1.Center;
            this.R2 = p - c.B2.Center;
            float r1crossN = Vec2D.ZValueFromCross(R1, c.Normal);
            float r2crossN = Vec2D.ZValueFromCross(R2, c.Normal);

            this.EffectiveMass = 1.0f / (B1.InverseMass + B2.InverseMass +
                r1crossN * r1crossN * B1.InverseInertia +
                r2crossN * r2crossN * B2.InverseInertia);

            this.ForceDirection = c.Normal;
            this.Bias = GetBias(settings, c, R1, R2);
        }

        private float GetBias(Settings s, CollisionInfo c, Vec2D r1, Vec2D r2)
        {
            Vec2D relativeVelocity = ResolutionHelper.GetRelativeVelocityBetweenAnchorPoints(c.B1, c.B2, r1, r2);

            // Relative velocity in normal direction
            float velocityInNormal = relativeVelocity * c.Normal;
            float restituion = System.Math.Min(c.B1.Restituion, c.B2.Restituion);

            float restitutionBias = -restituion * velocityInNormal;

            float biasFactor = s.DoPositionalCorrection ? s.PositionalCorrectionRate : 0.0f;
            float positionBias = biasFactor * s.InvDt * System.Math.Max(0, c.Depth - s.AllowedPenetration);

            return restitutionBias + positionBias;
        }
    }
}
