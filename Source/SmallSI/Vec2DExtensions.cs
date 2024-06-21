using OpenTK.Mathematics;
using Physics.Math;

namespace SmallSI
{
    internal static class Vec2DExtensions
    {
        public static Vector2 ToGrx(this Vec2D v)
        {
            return new Vector2(v.X, v.Y);
        }
    }
}
