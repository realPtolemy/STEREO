import socket
import lz4.block
import numpy as np
import open3d as o3d

HOST        = "127.0.0.1"
CLIENT_PORT = 3333

def receive_packet():
    print("recioving packets")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST, CLIENT_PORT))

    # —––––— 1) Read 1 datagram and extract the header from it.
    datagram, addr = sock.recvfrom(65535)

    compressed_size   = int.from_bytes(datagram[0:4], 'little')
    uncompressed_size = int.from_bytes(datagram[4:8], 'little')
    print(f"Header → comp_size={compressed_size:,}, raw_size={uncompressed_size:,}")

    # --- 2) Read remaning packets ---
    buf = bytearray(datagram[8:])
    while len(buf) < compressed_size:
        chunk, _ = sock.recvfrom(65535)
        buf.extend(chunk)
        # print(f"Got chunk {len(chunk):,} bytes (total {len(buf):,}/{compressed_size:,})")

    # —––––— 3) Acknowledge
    sock.sendto(b"Done", ("127.0.0.1", 3334))
    sock.close()

    print(f"Finished receiving packet: {len(buf):,} bytes")
    return bytes(buf), uncompressed_size

def deserialize_pc(packet, uncompressed_size, has_intensity=False):
    # 1) decompress
    raw = lz4.block.decompress(packet, uncompressed_size)
    # 2) interpret as floats
    floats = np.frombuffer(raw, dtype=np.float32)

    pts4   = floats.reshape(-1, 4)
    xyz    = pts4[:, :3]
    # 3) build Open3D cloud
    if has_intensity:
        intensity = pts4[:, 3]
        # normalize intensity to [0,1] and use as grayscale
        iv = (intensity - intensity.min()) / (intensity.ptp() + 1e-8)
        cols = np.stack([iv, iv, iv], axis=1)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        pcd.colors = o3d.utility.Vector3dVector(cols)
    else:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
    return pcd


# pcd = o3d.t.io.read_point_cloud("../../cloud_cluster_0000.pcd", format="pcd")

if __name__ == "__main__":
    packet, uncompressed_size = receive_packet()
    # if you serialized a PointXYZI cloud, pass has_intensity=True
    cloud = deserialize_pc(packet, uncompressed_size, has_intensity=False)
    o3d.visualization.draw_geometries([cloud])