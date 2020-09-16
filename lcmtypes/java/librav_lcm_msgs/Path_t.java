/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package librav_lcm_msgs;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class Path_t implements lcm.lcm.LCMEncodable
{
    public long waypoint_num;
    public librav_lcm_msgs.WayPoint_t waypoints[];
    public float const_velocity;
 
    public Path_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0xee74c1373a1dee5bL;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(librav_lcm_msgs.Path_t.class))
            return 0L;
 
        classes.add(librav_lcm_msgs.Path_t.class);
        long hash = LCM_FINGERPRINT_BASE
             + librav_lcm_msgs.WayPoint_t._hashRecursive(classes)
            ;
        classes.remove(classes.size() - 1);
        return (hash<<1) + ((hash>>63)&1);
    }
 
    public void encode(DataOutput outs) throws IOException
    {
        outs.writeLong(LCM_FINGERPRINT);
        _encodeRecursive(outs);
    }
 
    public void _encodeRecursive(DataOutput outs) throws IOException
    {
        outs.writeLong(this.waypoint_num); 
 
        for (int a = 0; a < this.waypoint_num; a++) {
            this.waypoints[a]._encodeRecursive(outs); 
        }
 
        outs.writeFloat(this.const_velocity); 
 
    }
 
    public Path_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public Path_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static librav_lcm_msgs.Path_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        librav_lcm_msgs.Path_t o = new librav_lcm_msgs.Path_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.waypoint_num = ins.readLong();
 
        this.waypoints = new librav_lcm_msgs.WayPoint_t[(int) waypoint_num];
        for (int a = 0; a < this.waypoint_num; a++) {
            this.waypoints[a] = librav_lcm_msgs.WayPoint_t._decodeRecursiveFactory(ins);
        }
 
        this.const_velocity = ins.readFloat();
 
    }
 
    public librav_lcm_msgs.Path_t copy()
    {
        librav_lcm_msgs.Path_t outobj = new librav_lcm_msgs.Path_t();
        outobj.waypoint_num = this.waypoint_num;
 
        outobj.waypoints = new librav_lcm_msgs.WayPoint_t[(int) waypoint_num];
        for (int a = 0; a < this.waypoint_num; a++) {
            outobj.waypoints[a] = this.waypoints[a].copy();
        }
 
        outobj.const_velocity = this.const_velocity;
 
        return outobj;
    }
 
}
