namespace Physics.CollisionResolution
{
    public class Settings
    {
        public float Dt;
        public float InvDt;
        public int IterationCount = 10;
        public float Gravity = 9.81f;        
        public float PositionalCorrectionRate = 0.2f; //0 = No correction; 1 = After one time step the collision is gone (this is the percentage by which the position is corrected per time step)
        public bool DoPositionalCorrection = true; //Should the collision be resolved after each TimeStep by shifting the position according to the collision normal?
        public float AllowedPenetration = 1.0f; //This is the number of pixels that two bodies can overlap without a correction pulse being applied. This creates stable RestingContacts
    }
}
