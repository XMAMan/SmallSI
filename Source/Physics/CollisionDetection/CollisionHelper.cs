namespace Physics.CollisionDetection
{
    internal static class CollisionHelper
    {
        public static CollisionInfo[] GetAllCollisions(List<RigidRectangle> bodies)
        {
            List<CollisionInfo> collisions = new List<CollisionInfo>();

            for (int i = 0; i < bodies.Count; i++)
                for (int j = i + 1; j < bodies.Count; j++)
                {
                    var b1 = bodies[i];
                    var b2 = bodies[j];
                    if (b1.InverseMass == 0 && b2.InverseMass == 0) continue; //No collisionpoint between non moveable bodies

                    if (BoundingCircleCollides(b1, b2))   //Broudphase-Test
                    {
                        var contacts = RectangleRectangleCollision.GetCollisionPoints(b1, b2); //Nearphase-Test
                        if (contacts.Any())
                            collisions.AddRange(contacts);
                    }
                }

            return collisions.ToArray();
        }

        internal static bool BoundingCircleCollides(RigidRectangle c1, RigidRectangle c2)
        {
            float d = (c1.Center - c2.Center).Length();
            return d < (c1.Radius + c2.Radius);
        }
    }
}
