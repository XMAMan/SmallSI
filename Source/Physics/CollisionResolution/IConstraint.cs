using Physics.Math;

namespace Physics.CollisionResolution
{
    internal interface IConstraint
    {
        RigidRectangle B1 { get; }
        RigidRectangle B2 { get; }
        Vec2D R1 { get; } //Lever arm from B1.Center to contact point
        Vec2D R2 { get; } //Lever arm from B2.Center to contact point
        float MinImpulse { get; }
        float MaxImpulse { get; }
        Vec2D ForceDirection { get; } //B2 is pressed in this direction (B1 is pressed in the opposite direction)
        float Bias { get; } //Target for the relative speed of the contact points
        float ImpulseMass { get; } //Conversion vector from the relative contact point velocity value to an impulse (corresponds to InverseK)
        float AccumulatedImpulse { get; set; }
    }
}
