using Physics.CollisionDetection;
using Physics.CollisionResolution;
using Physics.Math;

namespace Physics
{
    public class PhysicScene
    {
        public Settings Settings { get; set; } = new Settings();

        public List<RigidRectangle> Bodies { get; private set; } = new List<RigidRectangle>();

        public void TimeStep(float dt)
        {
            //Step 1: Get all collisionpoints
            var collisionsFromThisTimeStep = CollisionHelper.GetAllCollisions(Bodies);

            //Step 2: Create Constraints
            this.Settings.Dt = dt;
            this.Settings.InvDt = dt > 0.0f ? 1.0f / dt : 0.0f;
            List<IConstraint> constraints = new List<IConstraint>();
            foreach (var point in collisionsFromThisTimeStep)
            {
                constraints.Add(new NormalConstraint(this.Settings, point));
                constraints.Add(new FrictionConstraint(this.Settings, point));                
            }

            //Step 3: Apply Gravity-Force
            foreach (var body in Bodies)
            {
                //Body is not moveable
                if (body.InverseMass == 0)
                    continue;
                
                body.Velocity.Y += this.Settings.Gravity * dt; //v2 = v1 + a * dt     a = gravity
            }

            //Step 4: Apply Normal- and Friction-Force by using sequentiell impulses
            for (int i = 0; i < this.Settings.IterationCount; i++)
            {
                foreach (var c in constraints)
                {
                    Vec2D relativeVelocity = ResolutionHelper.GetRelativeVelocityBetweenAnchorPoints(c.B1, c.B2, c.R1, c.R2);
                    float velocityInForceDirection = relativeVelocity * c.ForceDirection; //this is the same as J*V
                    float impulse = c.EffectiveMass * (c.Bias - velocityInForceDirection); //lambda=forceLength*impulseDuration

                    // Clamp the accumulated impulse
                    float oldSum = c.AccumulatedImpulse;
                    c.AccumulatedImpulse = ResolutionHelper.Clamp(oldSum + impulse, c.MinImpulse, c.MaxImpulse);
                    impulse = c.AccumulatedImpulse - oldSum;

                    //Apply Impulse -> correct the velocity from B1 and B2
                    Vec2D impulseVec = impulse * c.ForceDirection;
                    c.B1.Velocity -= impulseVec * c.B1.InverseMass;
                    c.B1.AngularVelocity -= Vec2D.ZValueFromCross(c.R1, impulseVec) * c.B1.InverseInertia;
                    c.B2.Velocity += impulseVec * c.B2.InverseMass;
                    c.B2.AngularVelocity += Vec2D.ZValueFromCross(c.R2, impulseVec) * c.B2.InverseInertia;
                }
            }

            //Step 5: Move bodies
            foreach (var body in this.Bodies)
            {
                body.MoveCenter(dt * body.Velocity);
                body.Rotate(dt * body.AngularVelocity);
            }
        }
    }
}
