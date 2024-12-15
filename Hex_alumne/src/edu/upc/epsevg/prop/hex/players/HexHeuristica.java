package edu.upc.epsevg.prop.hex.players;

import edu.upc.epsevg.prop.hex.HexGameStatus;
import edu.upc.epsevg.prop.hex.PlayerType;
import java.awt.Point;
import java.util.*;

/**
 * Clase que implementa una heurística basada en la distancia mínima al objetivo
 * utilizando una variante del algoritmo de Dijkstra adaptado al juego Hex.
 */
public class HexHeuristica {

    /**
     * Evalúa el estado del juego. Si es un estado terminal, devuelve un valor
     * muy alto o muy bajo según el ganador. Si no es terminal, calcula las
     * distancias mínimas a la victoria para el jugador actual y el oponente, y
     * devuelve la diferencia.
     */
    public int evaluarEstado(HexGameStatus gameState) {
        if (gameState.isGameOver()) {
            // Si el juego terminó, asignamos valor extremo según el ganador.
            return (gameState.GetWinner() == gameState.getCurrentPlayer()) ? 100000000 : -100000000;
        }
        
        PlayerType currentPlayer = gameState.getCurrentPlayer();
        PlayerType opponent = (currentPlayer == PlayerType.PLAYER1) ? PlayerType.PLAYER2 : PlayerType.PLAYER1;
        ArrayList<Point> misFichas = obtenerFichasPropias(gameState, currentPlayer);
        int size = gameState.getSize();
        int score=0;
        // Calculamos distancia mínima para el jugador actual
        int distCurrent = calcularDistanciaMinima(gameState, currentPlayer);
        if(distCurrent==0) return 999999;
        // Calculamos distancia mínima para el oponente
        int distOpponent = calcularDistanciaMinima(gameState, opponent);
        if(distOpponent==0) return -999999;

        // Heurística = distOponente - distJugadorActual
         score += distOpponent - distCurrent;
         Point center = new Point(size / 2, size / 2);
         for (Point ficha : misFichas) {
            if (ficha.equals(center)) {
                score += 5;
            }
                score += evaluarConexionesVirtuales(gameState, ficha, opponent)- evaluarConexionesVirtuales(gameState, ficha, currentPlayer);
                //score += bloquearCaminoDelOponente(gameState, opponent);
         }
         
         return score;
    }
    /*private int bloquearCaminoDelOponente(HexGameStatus gameState, PlayerType opponent) {
    int size = gameState.getSize();
    int playerId = playerToId(opponent);
    int bloqueos = 0;

    // Obtén el camino más corto del oponente usando la distancia mínima
    List<Point> caminoCritico = calcularCaminoCritico(gameState, opponent);

    // Prioriza bloquear las celdas críticas en el camino
    for (Point p : caminoCritico) {
        if (gameState.getPos(p.x, p.y) == 0) { // Celda vacía
            bloqueos += 15; // Aumentamos la puntuación si es bloqueable
        } else if (gameState.getPos(p.x, p.y) == playerId) {
            bloqueos -= 10; // Penalizamos si ya está ocupada por el oponente
        }
    }
    return bloqueos;
}
    private List<Point> calcularCaminoCritico(HexGameStatus gameState, PlayerType p) {
    int size = gameState.getSize();
    int playerId = playerToId(p);
    List<Point> fuentes = new ArrayList<>();
    int[][] dist = new int[size][size];
    Point[][] prev = new Point[size][size]; // Para reconstruir el camino

    // Inicializamos distancias y fuentes como en Dijkstra
    if (p == PlayerType.PLAYER1) {
        for (int y = 0; y < size; y++) {
            if (gameState.getPos(0, y) != -playerId) { // No es del rival
                fuentes.add(new Point(0, y));
                dist[0][y] = 0;
            }
        }
    } else {
        for (int x = 0; x < size; x++) {
            if (gameState.getPos(x, 0) != -playerId) {
                fuentes.add(new Point(x, 0));
                dist[x][0] = 0;
            }
        }
    }

    // Usamos un PriorityQueue para Dijkstra
    PriorityQueue<Node> pq = new PriorityQueue<>(Comparator.comparingInt(n -> n.distance));
    for (Point f : fuentes) {
        pq.add(new Node(f.x, f.y, 0));
    }

    Point[] directions = {
        new Point(0, 1), new Point(1, 0), new Point(1, -1),
        new Point(0, -1), new Point(-1, 0), new Point(-1, 1)
    };

    // Ejecutamos Dijkstra y mantenemos las referencias previas
    while (!pq.isEmpty()) {
        Node current = pq.poll();

        for (Point d : directions) {
            int nx = current.x + d.x;
            int ny = current.y + d.y;
            if (nx < 0 || ny < 0 || nx >= size || ny >= size) continue;

            int cost = getCost(gameState, nx, ny, playerId);
            if (cost == Integer.MAX_VALUE) continue;

            int newDist = dist[current.x][current.y] + cost;
            if (newDist < dist[nx][ny]) {
                dist[nx][ny] = newDist;
                prev[nx][ny] = new Point(current.x, current.y);
                pq.add(new Node(nx, ny, newDist));
            }
        }
    }

    // Reconstruimos el camino más corto desde el borde opuesto
    List<Point> camino = new ArrayList<>();
    if (p == PlayerType.PLAYER1) {
        for (int y = 0; y < size; y++) {
            if (dist[size - 1][y] < Integer.MAX_VALUE) {
                Point curr = new Point(size - 1, y);
                while (curr != null) {
                    camino.add(curr);
                    curr = prev[curr.x][curr.y];
                }
                break;
            }
        }
    } else {
        for (int x = 0; x < size; x++) {
            if (dist[x][size - 1] < Integer.MAX_VALUE) {
                Point curr = new Point(x, size - 1);
                while (curr != null) {
                    camino.add(curr);
                    curr = prev[curr.x][curr.y];
                }
                break;
            }
        }
    }

    return camino;
}*/

    /**
     * Calcula la distancia mínima hasta la victoria para un jugador dado.
     * 
     * Para PLAYER1 (Rojo): conectar de arriba a abajo
     * Para PLAYER2 (Azul): conectar de izquierda a derecha
     *
     * @param gameState estado actual
     * @param p jugador para el cual calculamos la distancia
     * @return distancia mínima en "pasos" (colocaciones) necesarias. Si no hay
     *         camino, retorna un valor grande.
     */
    private int calcularDistanciaMinima(HexGameStatus gameState, PlayerType p) {
        int size = gameState.getSize();
        int playerId = (p == PlayerType.PLAYER1) ? 1 : -1;
        
        // Para PLAYER1 conectamos top (row=0) con bottom (row=size-1)
        // Para PLAYER2 conectamos left (col=0) con right (col=size-1)

        // Vamos a crear un grafo implícito representado por el tablero
        // y usar Dijkstra adaptado:
        // - Costo 0 si la celda es del mismo jugador
        // - Costo 1 si la celda está vacía
        // - No se puede pasar por celdas del rival.

        // Idea:
        // Para PLAYER1: 
        //   Lanzaremos Dijkstra desde un "super-nodo" virtual que conecta con todas las celdas de la fila superior (row=0) 
        //   que sean transitables (no del rival). 
        //   Tomamos la menor distancia a cualquier celda de la fila inferior (row=size-1).
        //
        // Para PLAYER2:
        //   Igual pero desde col=0 a col=size-1.

        // Creamos una lista de nodos fuente (según el jugador):
        List<Point> fuentes = new ArrayList<>();
        if (p == PlayerType.PLAYER1) {
            for (int y = 0; y < size; y++) {
                if (gameState.getPos(0, y) != -playerId) { // No es del rival
                    fuentes.add(new Point(0, y));
                }
            }
        } else {
            for (int x = 0; x < size; x++) {
                if (gameState.getPos(x, 0) != -playerId) {
                    fuentes.add(new Point(x, 0));
                }
            }
        }

        // Si no hay fuentes transitables, distancia muy grande:
        if (fuentes.isEmpty()) return 999999;

        int[][] dist = new int[size][size];
        for (int i = 0; i < size; i++) {
            Arrays.fill(dist[i], Integer.MAX_VALUE);
        }

        // Usamos un priority queue para Dijkstra
        PriorityQueue<Node> pq = new PriorityQueue<>(Comparator.comparingInt(n -> n.distance));

        // Inicializamos distancias desde las fuentes (0 para las fuentes si son del player, 1 si vacías)
        for (Point f : fuentes) {
            int cost = getCost(gameState, f.x, f.y, playerId);
            if (cost != Integer.MAX_VALUE) {
                dist[f.x][f.y] = cost; 
                pq.add(new Node(f.x, f.y, cost));
            }
        }

        // Movimientos posibles en Hex
        Point[] directions = {
            new Point(0, 1), new Point(1, 0), new Point(1, -1),
            new Point(0, -1), new Point(-1, 0), new Point(-1, 1)
        };

        // Dijkstra
        while (!pq.isEmpty()) {
            Node current = pq.poll();
            if (current.distance > dist[current.x][current.y]) continue;

            // Check si llegamos al borde contrario
            if ((p == PlayerType.PLAYER1 && current.x == size - 1) ||
                (p == PlayerType.PLAYER2 && current.y == size - 1)) {
                // Hemos conectado
                return current.distance;
            }

            for (Point d : directions) {
                int nx = current.x + d.x;
                int ny = current.y + d.y;
                if (nx < 0 || ny < 0 || nx >= size || ny >= size) continue;
                int c = getCost(gameState, nx, ny, playerId);
                if (c == Integer.MAX_VALUE) continue; // no se puede pasar
                int nd = current.distance + c;
                if (nd < dist[nx][ny]) {
                    dist[nx][ny] = nd;
                    pq.add(new Node(nx, ny, nd));
                }
            }
        }

        // No se encontró conexión
        return 999999;
    }

    /**
     * Devuelve el coste de pisar la celda (x,y) para el jugador playerId:
     * - 0 si la celda pertenece al playerId
     * - 1 si la celda está vacía
     * - Integer.MAX_VALUE si la celda pertenece al rival (infranqueable)
     */
    private int getCost(HexGameStatus gameState, int x, int y, int playerId) {
        int cell = gameState.getPos(x, y);
        if (cell == playerId) return 0;
        if (cell == 0) return 1;
        // cell == -playerId => rival
        return (cell == -playerId) ? Integer.MAX_VALUE : 1; 
    }

    private static class Node {
        int x, y, distance;
        Node(int x, int y, int dist) {
            this.x = x; this.y = y; this.distance = dist;
        }
    }
    
     private ArrayList<Point> obtenerFichasPropias(HexGameStatus gameState, PlayerType player) {
        ArrayList<Point> fichas = new ArrayList<>();
        int size = gameState.getSize();
        int playerId = playerToId(player);

        for (int x = 0; x < size; x++) {
            for (int y = 0; y < size; y++) {
                if (gameState.getPos(x, y) == playerId) {
                    fichas.add(new Point(x, y));
                }
            }
        }
        return fichas;
    }
     private int evaluarConexionesVirtuales(HexGameStatus gameState, Point ficha, PlayerType player) {
        int playerId = playerToId(player);
        Point[] directions = {
            new Point(2, -1),new Point(1,-1),new Point(1,0), new Point(1, 1),new Point(0,1),new Point(1,-0), new Point(-1, 2),
            new Point(-1,1),new Point(0,1),new Point(-2, 1),new Point(-1,1),new Point(-1,0), new Point(-1, -1),new Point(0,-1),
            new Point(-1,0), new Point(1, -2), new Point(0,-1),new Point(-1,-1)
        };
        int score = 0;

        for (int i=0;i<18;i=i+3) {
            
            Point diagonal = new Point(ficha.x +directions[i].x, ficha.y +directions[i].y);
            Point intermediari1 = new Point(ficha.x +directions[i+1].x, ficha.y +directions[i+1].y);
            Point intermediari2 = new Point(ficha.x +directions[i+2].x, ficha.y +directions[i+2].y);
            if(estaDentro(diagonal,gameState.getSize())){
                if(gameState.getPos(diagonal.x, diagonal.y) == playerId){
                    score +=20;
                }
            }
            if(estaDentro(intermediari1,gameState.getSize()) && estaDentro(intermediari2,gameState.getSize())){
                if(gameState.getPos(intermediari1.x, intermediari1.y) == playerId){
                    score +=20;
                } else if(gameState.getPos(intermediari2.x, intermediari2.y) == playerId){
                    score +=20;
                }
                 if(gameState.getPos(intermediari1.x, intermediari1.y) == 0 && gameState.getPos(intermediari2.x, intermediari2.y) == 0){
                    score +=10;
                }
                 else if (gameState.getPos(intermediari1.x, intermediari1.y) != playerId && gameState.getPos(intermediari2.x, intermediari2.y)  != playerId){
                    score =-10;
                }
            }
        }
        return score;
    }
     private boolean estaDentro(Point p, int size) {
        return p.x >= 0 && p.y >= 0 && p.x < size && p.y < size;
    }


    /**
     * Convierte PlayerType a ID numérico.
     */
    private int playerToId(PlayerType player) {
        return (player == PlayerType.PLAYER1) ? 1 : -1;
    }
}
