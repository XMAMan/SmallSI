using OpenTK.Graphics.OpenGL4;
using OpenTK.Windowing.Common;
using OpenTK.Windowing.Desktop;

namespace Graphic
{
    //Extends the GameWindow with 2D drawing functions
    //Usage: Derive from the class and implement the Draw method
    public abstract class GameWindow2D : GameWindow
    {
        private IDrawingContext drawingContext;
        private SolidQuadDrawer quadDrawer;

        public GameWindow2D(GameWindowSettings gameWindowSettings, NativeWindowSettings nativeWindowSettings)
           : base(gameWindowSettings, nativeWindowSettings)
        {
        }

        protected override void OnLoad()
        {
            base.OnLoad();
            this.quadDrawer = new SolidQuadDrawer();
            this.drawingContext = new Drawer2D(quadDrawer, () => SwapBuffers());
        }

        protected override void OnResize(ResizeEventArgs e)
        {
            base.OnResize(e);
            GL.Viewport(0, 0, Size.X, Size.Y);
            this.quadDrawer.HandleWindowSizeChanged(this.Size, this.ClientSize);
        }

        protected override void OnRenderFrame(FrameEventArgs e)
        {
            base.OnRenderFrame(e);
            Draw(this.drawingContext);
        }

        protected abstract void Draw(IDrawingContext context);
    }
}
