namespace Physics.Math
{
    internal class Matrix2x2
    {
        public readonly Vec2D Col1;
        public readonly Vec2D Col2;
        private Matrix2x2(Vec2D col1, Vec2D col2)
        {
            Col1 = col1;
            Col2 = col2;
        }

        public static Matrix2x2 Rotate(float angle)
        {
            float c = (float)System.Math.Cos(angle);
            float s = (float)System.Math.Sin(angle);

            return new Matrix2x2(new Vec2D(c, s), new Vec2D(-s, c));
        }

        public static Vec2D operator *(Matrix2x2 m, Vec2D v)
        {
            return new Vec2D(m.Col1.X * v.X + m.Col2.X * v.Y, m.Col1.Y * v.X + m.Col2.Y * v.Y);
        }
    }
}
