namespace Parallel_XPBD
{
    public class DistCon
    {
        public Connection[] Connections;
        public int[] Groups;
    }
    
    public struct Connection
    {
        public int ParticleOne;
        public int ParticleTwo;
        public float RestLenght;
        public float Compliance;
    }
}