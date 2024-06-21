namespace Physics.Math
{
    public class Vec2D
    {
        public float X { get; set; }
        public float Y { get; set; }

        public Vec2D(float x, float y)
        {
            X = x;
            Y = y;
        }

        public override string ToString()
        {
            return "[" + X.ToString() + ";" + Y.ToString() + "]";
        }

        public static Vec2D operator +(Vec2D v1, Vec2D v2)
        {
            return new Vec2D(v1.X + v2.X, v1.Y + v2.Y);
        }

        public static Vec2D operator -(Vec2D v1, Vec2D v2)
        {
            return new Vec2D(v1.X - v2.X, v1.Y - v2.Y);
        }
        public static Vec2D operator -(Vec2D v)
        {
            return -1 * v;
        }

        public static float operator *(Vec2D v1, Vec2D v2)
        {
            return v1.X * v2.X + v1.Y * v2.Y;
        }
        public static Vec2D operator *(Vec2D v, float f)
        {
            return new Vec2D(v.X * f, v.Y * f);
        }
        public static Vec2D operator *(float f, Vec2D v)
        {
            return new Vec2D(v.X * f, v.Y * f);
        }

        public static Vec2D operator /(Vec2D v, float f)
        {
            return new Vec2D(v.X / f, v.Y / f);
        }

        public float Length()
        {
            return (float)System.Math.Sqrt(X * X + Y * Y);
        }

        public Vec2D Normalize()
        {
            return this / Length();
        }

        public static float ZValueFromCross(Vec2D v1, Vec2D v2)
        {
            return v1.X * v2.Y - v2.X * v1.Y;
        }

        //Cross product between (v.X, v.Y, 0) and (0,0,z)
        public static Vec2D CrossWithZ(Vec2D v, float z)
        {
            return new Vec2D(z * v.Y, -z * v.X);
        }
    }
}
