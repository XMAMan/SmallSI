using OpenTK.Graphics.OpenGL4;
using OpenTK.Mathematics;
using System.Drawing;

namespace Graphic
{
    //Stores the vertex data of a 2D rectangle which is in the range of -0.5 to +0.5 and draws it using matrix shift
    internal class SolidQuadDrawer
    {
        private readonly float[] _vertices =
        {
            // Position         
             0.5f,  0.5f, 0.0f, // top right
             0.5f, -0.5f, 0.0f, // bottom right
            -0.5f, -0.5f, 0.0f, // bottom left
            -0.5f,  0.5f, 0.0f  // top left
        };

        private readonly uint[] _indices =
        {
            0, 1, 3,
            1, 2, 3
        };

        private int _vertexBufferObject;
        private int _vertexArrayObject;
        private Shader _shader;
        private int _elementBufferObject;



        public SolidQuadDrawer()
        {
            _vertexBufferObject = GL.GenBuffer();
            GL.BindBuffer(BufferTarget.ArrayBuffer, _vertexBufferObject);
            GL.BufferData(BufferTarget.ArrayBuffer, _vertices.Length * sizeof(float), _vertices, BufferUsageHint.StaticDraw);
            _vertexArrayObject = GL.GenVertexArray();
            GL.BindVertexArray(_vertexArrayObject);

            _shader = new Shader("Shaders/SolidQuad/shader.vert", "Shaders/SolidQuad/shader.frag");
            _shader.Use();

            var vertexLocation = _shader.GetAttribLocation("aPosition");
            GL.EnableVertexAttribArray(vertexLocation);
            GL.VertexAttribPointer(vertexLocation, 3, VertexAttribPointerType.Float, false, 3 * sizeof(float), 0);
            GL.EnableVertexAttribArray(0);
            _elementBufferObject = GL.GenBuffer();
            GL.BindBuffer(BufferTarget.ElementArrayBuffer, _elementBufferObject);
            GL.BufferData(BufferTarget.ElementArrayBuffer, _indices.Length * sizeof(uint), _indices, BufferUsageHint.StaticDraw);
        }

        private Vector2i windowSize = new Vector2i(100, 100);
        private Vector2i clientSize = new Vector2i(100, 100);
        private Matrix4 projectionMatrix = Matrix4.Identity;

        public void HandleWindowSizeChanged(Vector2i windowSize, Vector2i clientSize)
        {
            this.windowSize = windowSize;
            this.clientSize = clientSize;
            this.projectionMatrix = Matrix4.CreateOrthographicOffCenter(0, this.windowSize.X, 0, this.windowSize.Y, -100, +100);
        }

        public void DrawAxialRectangle(RectangleF rec, Color color)
        {
            GL.BindVertexArray(_vertexArrayObject);

            var transform = Matrix4.Identity;
            transform = transform * Matrix4.CreateScale(rec.Width, rec.Height, 1);
            transform = transform * Matrix4.CreateTranslation(rec.X + rec.Width / 2, clientSize.Y - rec.Y - rec.Height / 2, 0.0f);
            _shader.Use();

            _shader.SetMatrix4("transform", transform);
            _shader.SetMatrix4("projection", this.projectionMatrix);
            _shader.SetVector4("color", new Vector4(color.R / 255f, color.G / 255f, color.B / 255, color.A / 255f));

            GL.DrawElements(PrimitiveType.Triangles, _indices.Length, DrawElementsType.UnsignedInt, 0);
        }

        public void DrawRotatedRectangle(Vector2 center, float width, float height, float angle, Color color)
        {
            GL.BindVertexArray(_vertexArrayObject);

            var transform = Matrix4.Identity;

            transform = transform * Matrix4.CreateScale(width, height, 1);
            transform = transform * Matrix4.CreateRotationZ(angle);
            transform = transform * Matrix4.CreateTranslation(center.X, clientSize.Y - center.Y, 0.0f);

            _shader.Use();

            _shader.SetMatrix4("transform", transform);
            _shader.SetMatrix4("projection", this.projectionMatrix);
            _shader.SetVector4("color", new Vector4(color.R / 255f, color.G / 255f, color.B / 255, color.A / 255f));

            GL.DrawElements(PrimitiveType.Triangles, _indices.Length, DrawElementsType.UnsignedInt, 0);
        }

        public void DrawLine(Vector2 p1, Vector2 p2, float lineWidth, Color color)
        {
            var dir = p2 - p1;
            float length = dir.Length;
            Vector2 center = (p1 + p2) / 2;
            float angle = (float)(Math.PI - Math.Atan2(dir.Y, dir.X));

            DrawRotatedRectangle(center, length, lineWidth, angle, color);
        }
    }
}
