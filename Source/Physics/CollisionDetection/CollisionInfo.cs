using Physics.Math;

namespace Physics.CollisionDetection
{
    internal class CollisionInfo
    {
        public Vec2D Start { get; }  //Collisionpoint from RigidBody1
        public Vec2D End { get; }    //Collisionpoint from RigidBody2
        public Vec2D Normal { get; } //Normal from rectangle side
        public float Depth { get; }     //Distance between Start and End
        public RigidRectangle B1 { get; }
        public RigidRectangle B2 { get; }

        internal CollisionInfo(Vec2D start, Vec2D normal, float depth, RigidRectangle b1, RigidRectangle b2)
            : this(start, start + normal * depth, normal, depth, b1, b2)
        {
        }

        protected CollisionInfo(Vec2D start, Vec2D end, Vec2D normal, float depth, RigidRectangle b1, RigidRectangle b2)
        {
            this.Start = start;
            this.End = end;
            this.Normal = normal;
            this.Depth = depth;
            B1 = b1;
            B2 = b2;
        }
    }
}
