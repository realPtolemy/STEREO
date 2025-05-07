import socket
import lz4.block
import numpy as np
import open3d as o3d
import time
import queue
import threading

HOST        = "127.0.0.1"
CLIENT_PORT = 3333
cloud_queue = queue.Queue()

def receive_packet(sock):
    print("recioving packets")
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
    # sock.sendto(b"Done", ("127.0.0.1", 3334))

    print(f"Finished receiving packet: {len(buf):,} bytes")
    return bytes(buf), uncompressed_size

def deserialize_pc(packet, uncompressed_size, has_intensity):
    raw = lz4.block.decompress(packet, uncompressed_size)
    floats = np.frombuffer(raw, dtype=np.float32)

    pts4   = floats.reshape(-1, 4)
    xyz    = pts4[:, :3]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    # if has_intensity:
    #     intensity = pts4[:, 3]
    #     iv = (intensity - intensity.min()) / (np.ptp(intensity) + 1e-8)
    #     cols = np.stack([iv, iv, iv], axis=1)
        # pcd.colors = o3d.utility.Vector3dVector(cols)
    return pcd


# pcd = o3d.t.io.read_point_cloud("../../cloud_cluster_0000.pcd", format="pcd")

def visualizer_thread(sock):
    vis = o3d.visualization.Visualizer()
    vis.create_window("PointCloud", width=1024, height=768)

    pcd = o3d.geometry.PointCloud()
    vis.add_geometry(pcd)
    first = True

    while True:
        try:
            new_pcd = cloud_queue.get(timeout=0.1)

            np_pts = np.asarray(new_pcd.points)
            np_cols = np.asarray(new_pcd.colors) if new_pcd.has_colors() else None

            if len(pcd.points) == 0:
                pcd.points = o3d.utility.Vector3dVector(np_pts)
                if np_cols is not None:
                    pcd.colors = o3d.utility.Vector3dVector(np_cols)
            else:
                np.asarray(pcd.points)[:] = np_pts
                if np_cols is not None:
                    np.asarray(pcd.colors)[:] = np_cols

            vis.update_geometry(pcd)
            if first:
                vis.reset_view_point(True)
                first = False
        except queue.Empty:
            pass

        vis.poll_events()
        vis.update_renderer()
        time.sleep(0.05)
        sock.sendto(b"Done", ("127.0.0.1", 3334))
    vis.destroy_window()
    sock.close()



def receiver_thread(sock):

    while True:
        packet, size = receive_packet(sock)
        cloud = deserialize_pc(packet, size, has_intensity=True)
        cloud_queue.put(cloud)

        if cloud_queue.qsize() < 2:
            cloud_queue.put(cloud)

        time.sleep(0.05)

if __name__ == "__main__":
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST, CLIENT_PORT))
    threading.Thread(target=receiver_thread, args=(sock,), daemon=True).start()
    visualizer_thread(sock)