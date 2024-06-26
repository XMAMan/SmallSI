﻿using Physics.CollisionDetection;
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
            //Step 1: Add gravity-force to each body
            foreach (var body in this.Bodies)
            {
                //Body is not moveable
                if (body.InverseMass == 0)
                    continue; 

                body.Force += new Vec2D(0, this.Settings.Gravity) / body.InverseMass;
            }

            //Step 2: Get all collisionpoints
            var collisionsFromThisTimeStep = CollisionHelper.GetAllCollisions(Bodies);

            //Step 3: Create Constraints
            this.Settings.Dt = dt;
            this.Settings.InvDt = dt > 0.0f ? 1.0f / dt : 0.0f;
            List<IConstraint> constraints = new List<IConstraint>();
            foreach (var point in collisionsFromThisTimeStep)
            {
                constraints.Add(new NormalConstraint(this.Settings, point));
                constraints.Add(new FrictionConstraint(this.Settings, point));                
            }

            //Step 4: Apply Gravity-Force
            foreach (var body in Bodies)
            {
                body.Velocity.X += body.InverseMass * body.Force.X * dt;
                body.Velocity.Y += body.InverseMass * body.Force.Y * dt;
                body.AngularVelocity += body.InverseInertia * body.Torque * dt;
            }

            //Step 5: Apply Normal- and Friction-Force by using sequentiell impulses
            for (int i = 0; i < this.Settings.IterationCount; i++)
            {
                foreach (var c in constraints)
                {
                    Vec2D relativeVelocity = ResolutionHelper.GetRelativeVelocityBetweenAnchorPoints(c.B1, c.B2, c.R1, c.R2);
                    float velocityInForceDirection = relativeVelocity * c.ForceDirection;
                    float impulse = c.ImpulseMass * (c.Bias - velocityInForceDirection);

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

            //Step 6: Move bodies
            foreach (var body in this.Bodies)
            {
                body.MoveCenter(dt * body.Velocity);
                body.Rotate(dt * body.AngularVelocity);

                //Reset for the next TimeStep-call the external force
                body.Force = new Vec2D(0, 0);
                body.Torque = 0;
            }
        }
    }
}
