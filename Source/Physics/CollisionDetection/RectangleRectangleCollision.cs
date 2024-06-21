using Physics.Math;

namespace Physics.CollisionDetection
{
    internal static class RectangleRectangleCollision
    {
        struct SupportStruct
        {
            internal SupportStruct(float supportPointDist)
            {
                this.MaxDistance = supportPointDist;
            }

            internal List<byte> SupportPoints = new List<byte>(); //here the index is from a corner point of a rectangle
            internal float MaxDistance = 0;
        }

        //Return all the points from rectangle r that lie behind the edge p1OnEdge-p2OnEdge
        private static SupportStruct FindSupportPoint(RigidRectangle r, Vec2D dir, Vec2D p1OnEdge, Vec2D p2OnEdge)
        {
            var tmpSupport = new SupportStruct(-9999999);

            //check each vector of other object
            for (byte i = 0; i < r.Vertex.Length; i++)
            {
                //the longest project length
                Vec2D p1ToRi = r.Vertex[i] - p1OnEdge;
                float normalCheck = p1ToRi * dir;

                if (normalCheck < 0) continue; //Check that it has entered the "dir" side

                Vec2D p1ToP2 = p2OnEdge - p1OnEdge;
                if (p1ToRi * p1ToP2 < 0) continue; //Check that it is in front of the left edge
                Vec2D p2ToRi = r.Vertex[i] - p2OnEdge;
                if (p2ToRi * (-p1ToP2) < 0) continue; //Check that it is in front of the right edge

                if (normalCheck > tmpSupport.MaxDistance) tmpSupport.MaxDistance = normalCheck;
                tmpSupport.SupportPoints.Add(i);
            }

            return tmpSupport;
        }

        //Returns all the points of r2 that lie in r1. The r1 face normal is chosen where the points from r2 has penetrated the least
        private static CollisionInfo[] FindAxisLeastPenetration(RigidRectangle r1, RigidRectangle r2)
        {
            float bestDistance = 999999;
            int r1Face = -1;
            byte[] supportPoints = null;

            for (int i = 0; i < r1.FaceNormal.Length; i++)
            {
                // Retrieve a face normal from A
                var n = r1.FaceNormal[i];

                // use -n as direction and the vectex on edge i as point on edge

                // find the support on B
                // the point has longest distance with edge i 
                var tmpSupport = FindSupportPoint(r2, -n, r1.Vertex[i], r1.Vertex[(i + 1) % 4]);

                //SAT says if one side from r1 has no support-Point on r2, then there is no collision
                if (tmpSupport.SupportPoints == null) return null;

                //get the shortest support point depth
                if (tmpSupport.MaxDistance < bestDistance)
                {
                    bestDistance = tmpSupport.MaxDistance;
                    r1Face = i;
                    supportPoints = tmpSupport.SupportPoints.ToArray();
                }
            }

            //all four directions have support point. That means at least one point from r2 lies inside from r1
            return supportPoints.Select(r2Edge => new CollisionInfo(r2.Vertex[r2Edge] + r1.FaceNormal[r1Face] * bestDistance, r1.FaceNormal[r1Face], bestDistance, r1, r2)).ToArray();
        }

        internal static CollisionInfo[] GetCollisionPoints(RigidRectangle r1, RigidRectangle r2)
        {
            List<CollisionInfo> contacts = new List<CollisionInfo>();

            var collisionInfoR1 = FindAxisLeastPenetration(r1, r2);
            if (collisionInfoR1 != null)
                contacts.AddRange(collisionInfoR1.Select(x => new CollisionInfo(x.Start - x.Normal * x.Depth, x.Normal, x.Depth, r1, r2)));

            var collisionInfoR2 = FindAxisLeastPenetration(r2, r1);
            if (collisionInfoR2 != null)
                contacts.AddRange(collisionInfoR2.Select(x => new CollisionInfo(x.Start, -x.Normal, x.Depth, r1, r2)));

            if (contacts.Any() == false) return new CollisionInfo[0];

            //remove dupplicated collisionpoints (Happens if two cubes are stacked)
            var withoutDoubles = contacts
                .GroupBy(x => x.Start.ToString() + x.Depth.ToString())
                .Select(x => x.First())
                .ToArray();

            return withoutDoubles;
        }
    }
}
