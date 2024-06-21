using OpenTK.Mathematics;
using OpenTK.Windowing.Common;
using OpenTK.Windowing.Desktop;

namespace SmallSI
{
    public static class Program
    {
        private static void Main()
        {
            var nativeWindowSettings = new NativeWindowSettings()
            {
                ClientSize = new Vector2i(900, 550),
                Title = "2D Rigidbody physics",                
                Flags = ContextFlags.ForwardCompatible, // This is needed to run on macos
            };
            using (var window = new PhysicWindow(GameWindowSettings.Default, nativeWindowSettings))
            {
                window.Run();
            }
        }
    }
}