using Physics.Math;

namespace Physics
{
    public class RigidRectangle
    {
        public Vec2D Size { get; private set; }

        public Vec2D Center { get; private set; } //Position of the Center of gravity
        public float Angle { get; private set; } //Oriantation around the Z-Aches with rotationpoint=Center [0..2PI]
        public Vec2D Velocity { get; set; } //Velocity from the Center-Point
        public float AngularVelocity { get; set; }

        public float InverseMass { get; private set; } //1 / Mass
        public float InverseInertia { get; private set; }

        public float Restituion { get; set; } = 0.2f;
        public float Friction { get; set; } = 0.1f;

        public float Radius { get; private set; } //Used for boundingbox-Test

        //0--TopLeft;1--TopRight;2--BottomRight;3--BottomLeft
        public Vec2D[] Vertex { get; private set; }
        private Vec2D[] vertexLocal;

        //0--Top;1--Right;2--Bottom;3--Left
        public Vec2D[] FaceNormal { get; private set; }

        public RigidRectangle(Vec2D center, Vec2D size, float angle, float density, float restituion, float friction)
        {
            this.Center = center;
            this.Size = size;
            this.Angle = 0; //This Property gets his value by calling Rotate
            this.Velocity = new Vec2D(0, 0);
            this.AngularVelocity = 0;
            

            float mass = size.X * size.Y * density;
            this.InverseMass = density == float.MaxValue ? 0 : 1 / mass;
            this.InverseInertia = InverseMass == 0 ? 0 : 1.0f / (mass * (size.X * size.X + size.Y * size.Y) / 12f);
            
            this.Radius = (float)System.Math.Sqrt(size.X * size.X + size.Y * size.Y) / 2;

            this.vertexLocal = new Vec2D[]
            {
                new Vec2D(-size.X / 2, -size.Y / 2), //TopLeft
                new Vec2D(+size.X / 2, -size.Y / 2), //TopRight
                new Vec2D(+size.X / 2, +size.Y / 2), //BottomRight
                new Vec2D(-size.X / 2, +size.Y / 2), //BottomLeft
            };

            this.Vertex = new Vec2D[vertexLocal.Length];

            this.Restituion = restituion;
            this.Friction = friction;

            Rotate(angle);
        }

        public void MoveCenter(Vec2D v)
        {
            for (int i = 0; i < Vertex.Length; i++)
            {
                Vertex[i] += v;
            }

            Center += v;
        }

        public void Rotate(float angle)
        {
            Angle += angle;

            var rotateToWorld = Matrix2x2.Rotate(Angle);
            for (int i = 0; i < vertexLocal.Length; i++)
            {
                Vertex[i] = Center + rotateToWorld * vertexLocal[i];
            }

            UpdateFaceNormal();
        }

        private void UpdateFaceNormal()
        {
            FaceNormal = new Vec2D[]
            {
                (Vertex[1] - Vertex[2]).Normalize(), //Top
                (Vertex[2] - Vertex[3]).Normalize(), //Right
                (Vertex[3] - Vertex[0]).Normalize(), //Bottom
                (Vertex[0] - Vertex[1]).Normalize(), //Left
            };
        }
    }
}
