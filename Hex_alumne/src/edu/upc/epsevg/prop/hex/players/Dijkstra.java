package edu.upc.epsevg.prop.hex.utils;

import java.awt.Point;
import java.util.*;

/**
 * Implementación del algoritmo de Dijkstra para el juego Hex.
 */
public class Dijkstra {

    private int[][] board; // Representación del tablero (0: vacío, 1/2: jugadores)
    private int size; // Tamaño del tablero
    private int[][] distances; // Distancias mínimas desde el nodo inicial
    private boolean[][] visited; // Seguimiento de nodos visitados
    private Point[] directions = { // Movimientos posibles en un hexágono
            new Point(0, 1), new Point(1, 0), new Point(1, -1),
            new Point(0, -1), new Point(-1, 0), new Point(-1, 1)
    };

    /**
     * Constructor de la clase Dijkstra.
     * 
     * @param board Tablero de juego
     */
    public Dijkstra(int[][] board) {
        this.board = board;
        this.size = board.length;
        this.distances = new int[size][size];
        this.visited = new boolean[size][size];
        for (int i = 0; i < size; i++) {
            Arrays.fill(distances[i], Integer.MAX_VALUE);
        }
    }

    /**
     * Calcula las distancias mínimas desde un nodo inicial.
     * 
     * @param start Nodo inicial
     */
    public void calculateDistances(Point start) {
        PriorityQueue<Node> queue = new PriorityQueue<>(Comparator.comparingInt(node -> node.distance));
        queue.add(new Node(start, 0));
        distances[start.x][start.y] = 0;

<<<<<<< HEAD
=======
        //System.out.println("Iniciando Dijkstra desde: " + start);

>>>>>>> c3fd4fc (Tfdjssh)
        while (!queue.isEmpty()) {
            Node current = queue.poll();
            Point pos = current.position;

            if (visited[pos.x][pos.y]) {
                continue;
            }
            visited[pos.x][pos.y] = true;

<<<<<<< HEAD
=======
            //System.out.println("Visitando nodo: " + pos + " con distancia acumulada: " + current.distance);

>>>>>>> c3fd4fc (Tfdjssh)
            for (Point dir : directions) {
                Point neighbor = new Point(pos.x + dir.x, pos.y + dir.y);
                if (isValid(neighbor)) {
                    int newDist = distances[pos.x][pos.y] + 1; // Asume peso 1
<<<<<<< HEAD
                    if (newDist < distances[neighbor.x][neighbor.y]) {
                        distances[neighbor.x][neighbor.y] = newDist;
                        queue.add(new Node(neighbor, newDist));
=======
                    //System.out.println("  Evaluando vecino: " + neighbor + " con nueva distancia: " + newDist);
                    if (newDist < distances[neighbor.x][neighbor.y]) {
                        distances[neighbor.x][neighbor.y] = newDist;
                        queue.add(new Node(neighbor, newDist));
                        //System.out.println("    Actualizando distancia de " + neighbor + " a " + newDist);
>>>>>>> c3fd4fc (Tfdjssh)
                    }
                }
            }
        }
    }

<<<<<<< HEAD
=======

>>>>>>> c3fd4fc (Tfdjssh)
    /**
     * Verifica si un nodo es válido y no visitado.
     * 
     * @param p Nodo a verificar
     * @return true si es válido, false de lo contrario
     */
    private boolean isValid(Point p) {
        return p.x >= 0 && p.x < size && p.y >= 0 && p.y < size && !visited[p.x][p.y];
    }

    /**
     * Devuelve la distancia mínima a un nodo.
     * 
     * @param end Nodo final
     * @return Distancia mínima
     */
    public int getDistance(Point end) {
        return distances[end.x][end.y];
    }

    /**
     * Clase auxiliar para representar nodos en la cola de prioridad.
     */
    private static class Node {
        Point position;
        int distance;

        public Node(Point position, int distance) {
            this.position = position;
            this.distance = distance;
        }
    }
<<<<<<< HEAD
}
=======
}
>>>>>>> c3fd4fc (Tfdjssh)
