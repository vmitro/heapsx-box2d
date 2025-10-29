package heapsx.box2d.ll;

import haxe.io.Bytes;

/**
 * Helper abstract to access HashLink's raw bytes without @:privateAccess.
 * This provides a clean way to get the underlying hl.Bytes for native calls.
 */
@:forward
abstract HLBytes(Bytes) from Bytes to Bytes {
    /**
     * Allocate a new byte buffer
     */
    public static inline function alloc(size: Int): HLBytes {
        return Bytes.alloc(size);
    }

    /**
     * Get the raw hl.Bytes pointer for native calls
     */
    public var raw(get, never): hl.Bytes;
    inline function get_raw(): hl.Bytes {
        return @:privateAccess this.b;
    }

    /**
     * Set a float32 value at the specified byte offset
     */
    public inline function setF32(offset: Int, value: Float): Void {
        this.setFloat(offset, value);
    }

    /**
     * Get a float32 value at the specified byte offset
     */
    public inline function getF32(offset: Int): Float {
        return this.getFloat(offset);
    }

    /**
     * Set a float64 (double) value at the specified byte offset
     */
    public inline function setF64(offset: Int, value: Float): Void {
        this.setDouble(offset, value);
    }

    /**
     * Get a float64 (double) value at the specified byte offset
     */
    public inline function getF64(offset: Int): Float {
        return this.getDouble(offset);
    }
}
