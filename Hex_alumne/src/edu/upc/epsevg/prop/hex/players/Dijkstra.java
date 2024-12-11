package edu.upc.epsevg.prop.hex.players;

import edu.upc.epsevg.prop.hex.HexGameStatus;
import edu.upc.epsevg.prop.hex.PlayerType;
import java.util.*;

public class Dijkstra {

    public static int getShortestPath(HexGameStatus s, PlayerType color) {
        int size = s.getSize();
        
        boolean isPlayer1 = (color == PlayerType.PLAYER1);

        int[][] dist = new int[size][size];
        for (int i=0; i<size; i++){
            Arrays.fill(dist[i], Integer.MAX_VALUE);
        }

        PriorityQueue<Node> pq = new PriorityQueue<>(Comparator.comparingInt(n->n.dist));

        // Añadimos nodos iniciales según el jugador
        if (isPlayer1) {
            // PLAYER1 conecta de arriba (0) a abajo (size-1)
            for (int col = 0; col < size; col++) {
                if (s.getPos(0, col) != -1) { 
                    dist[0][col] = 0;
                    pq.add(new Node(0, col, 0));
                }
            }
        } else {
            // PLAYER2 conecta de izquierda (0) a derecha (size-1)
            for (int row = 0; row < size; row++) {
                if (s.getPos(row, 0) != 1) {
                    dist[row][0] = 0;
                    pq.add(new Node(row, 0, 0));
                }
            }
        }

        int[][] dirs = {
            {-1,0}, {1,0}, {0,-1}, {0,1}, {-1,1}, {1,-1}
        };

        while (!pq.isEmpty()) {
            Node current = pq.poll();
            if (current.dist > dist[current.x][current.y]) continue;

            if (isPlayer1 && current.x == size-1) {
                // PLAYER1 alcanzó el lado opuesto (abajo)
                return current.dist;
            } else if (!isPlayer1 && current.y == size-1) {
                // PLAYER2 alcanzó el lado opuesto (derecha)
                return current.dist;
            }

            for (int[] d : dirs) {
                int nx = current.x + d[0];
                int ny = current.y + d[1];
                if (nx >=0 && nx<size && ny>=0 && ny<size) {
                    int cell = s.getPos(nx, ny);
                    boolean blocked = (isPlayer1 && cell == -1) || (!isPlayer1 && cell == 1);
                    if (!blocked) {
                        int ndist = current.dist + 1;
                        if (ndist < dist[nx][ny]) {
                            dist[nx][ny] = ndist;
                            pq.add(new Node(nx, ny, ndist));
                        }
                    }
                }
            }
        }

        return 999999;
    }

    private static class Node {
        int x, y, dist;
        Node(int x, int y, int dist) {
            this.x=x; this.y=y; this.dist=dist;
        }
    }
}

