package edu.upc.epsevg.prop.hex.players;

import edu.upc.epsevg.prop.hex.HexGameStatus;
import edu.upc.epsevg.prop.hex.PlayerType;
import edu.upc.epsevg.prop.hex.utils.Dijkstra;
import java.awt.Point;
import java.util.ArrayList;

public class HexHeuristica {

    public int evaluarEstado(HexGameStatus gameState) {
        PlayerType currentPlayer = gameState.getCurrentPlayer();
        PlayerType rival = (currentPlayer == PlayerType.PLAYER1) ? PlayerType.PLAYER2 : PlayerType.PLAYER1;
        int size = gameState.getSize();
        int score = 0;

        int myId = (currentPlayer == PlayerType.PLAYER1) ? 1 : -1;
        int rivalId = (rival == PlayerType.PLAYER1) ? 1 : -1;

        ArrayList<Point> misFichas = obtenerFichasPropias(gameState, myId);
        ArrayList<Point> fichasVisitadas = new ArrayList<>();

        Point center = new Point(size / 2, size / 2);

        for (Point ficha : misFichas) {
            if (fichasVisitadas.contains(ficha)) continue;

            if (ficha.equals(center)) {
                score += 1000;
            }

            score += evaluarPosicionesEstrategicas(gameState, myId, rivalId, ficha, size);

            int distanciaBorde1 = calcularDistanciaBordes(gameState, myId, ficha, currentPlayer == PlayerType.PLAYER1);
            int distanciaBorde2 = calcularDistanciaBordes(gameState, myId, ficha, !(currentPlayer == PlayerType.PLAYER1));

            if (distanciaBorde1 < 10000 / 2) score += 500;
            if (distanciaBorde2 < 10000 / 2) score += 500;

            score += (10000 - distanciaBorde1) + (10000 - distanciaBorde2);

            fichasVisitadas.add(ficha);
        }

        score += bloquearRival(gameState, rivalId, size);

<<<<<<< HEAD
        return score;
    }

    private ArrayList<Point> obtenerFichasPropias(HexGameStatus gameState, int playerId) {
        ArrayList<Point> fichas = new ArrayList<>();
        int size = gameState.getSize();
        for (int x = 0; x < size; x++) {
            for (int y = 0; y < size; y++) {
                if (gameState.getPos(x, y) == playerId) {
                    fichas.add(new Point(x, y));
                }
            }
        }
        return fichas;
    }

    private int evaluarPosicionesEstrategicas(HexGameStatus gameState, int myId, int rivalId, Point ficha, int size) {
        int score = 0;
        Point[] diagonales = {
            new Point(2, -1), new Point(1, 1), new Point(-1, 2),
            new Point(-2, 1), new Point(-1, -1), new Point(1, -2)
        };

        for (Point dir : diagonales) {
            Point vecino = new Point(ficha.x + dir.x, ficha.y + dir.y);
            if (isInsideBoard(vecino, size)) {
                int cell = gameState.getPos(vecino.x, vecino.y);
                if (cell == myId) {
                    score += 50;
                    Point inter1 = new Point(ficha.x + dir.x / 2, ficha.y + dir.y / 2);
                    Point inter2 = new Point(ficha.x + dir.x / 2 + dir.y / 2, ficha.y + dir.y / 2 - dir.x / 2);

                    if (isInsideBoard(inter1, size) && isInsideBoard(inter2, size)) {
                        int cell1 = gameState.getPos(inter1.x, inter1.y);
                        int cell2 = gameState.getPos(inter2.x, inter2.y);

                        if ((cell1 == 0 && cell2 == 0) || (cell1 == myId && cell2 == 0) || (cell1 == 0 && cell2 == myId)) {
                            score += 100;
                        } else if (cell1 == rivalId && cell2 == rivalId) {
                            score -= 50;
                        } else if (cell1 == myId && cell2 == myId) {
                            score += 200;
                        }
                    }
                } else if (cell == rivalId) {
                    score -= 20;
                }
            }
        }
        return score;
    }

    private int calcularDistanciaBordes(HexGameStatus gameState, int playerId, Point ficha, boolean vertical) {
        int size = gameState.getSize();
        boolean[][] visitadas = new boolean[size][size];
        ArrayList<Point> cola = new ArrayList<>();
        cola.add(ficha);
        visitadas[ficha.x][ficha.y] = true;

        int distancia = 0;
        while (!cola.isEmpty()) {
            ArrayList<Point> siguienteNivel = new ArrayList<>();

            for (Point actual : cola) {
                if (haLlegadoABorde(actual, vertical, size)) {
                    return distancia;
                }

                for (Point vecino : gameState.getNeigh(actual)) {
                    if (vecino != null && isInsideBoard(vecino, size) && !visitadas[vecino.x][vecino.y] && gameState.getPos(vecino.x, vecino.y) != -playerId) {
                        siguienteNivel.add(vecino);
                        visitadas[vecino.x][vecino.y] = true;
                    }
                }
            }

            cola = siguienteNivel;
            distancia++;
        }

        return Integer.MAX_VALUE;
    }

    private boolean haLlegadoABorde(Point punto, boolean vertical, int size) {
        return vertical ? (punto.x == 0 || punto.x == size - 1) : (punto.y == 0 || punto.y == size - 1);
    }

    private int bloquearRival(HexGameStatus gameState, int rivalId, int size) {
        int bloqueos = 0;
        Point[] direcciones = {
            new Point(1, 0), new Point(0, 1), new Point(1, -1),
            new Point(-1, 0), new Point(0, -1), new Point(-1, 1)
        };

        for (int x = 0; x < size; x++) {
            for (int y = 0; y < size; y++) {
                if (gameState.getPos(x, y) == rivalId) {
                    for (Point dir : direcciones) {
                        Point vecino = new Point(x + dir.x, y + dir.y);
                        if (isInsideBoard(vecino, size) && gameState.getPos(vecino.x, vecino.y) == 0) {
                            bloqueos += 500; // Penaliza espacios críticos alrededor del rival
                        }
=======
        // Usar Dijkstra para calcular la distancia mínima hacia los bordes del jugador
        int[][] board = construirTablero(gameState, size);
        Dijkstra dijkstra = new Dijkstra(board);

        if (currentPlayer == PlayerType.PLAYER1) {
            // Conectar bordes horizontales (izquierda a derecha)
            for (int x = 0; x < size; x++) {
                dijkstra.calculateDistances(new Point(x, 0)); // Desde el borde izquierdo
                for (int y = 0; y < size; y++) {
                    int distancia = dijkstra.getDistance(new Point(x, size - 1)); // Hasta el borde derecho
                    if (distancia != Integer.MAX_VALUE) {
                        score += 1000 - distancia; // Premia caminos cortos
                    }
                }
            }
        } else {
            // Conectar bordes verticales (arriba a abajo)
            for (int y = 0; y < size; y++) {
                dijkstra.calculateDistances(new Point(0, y)); // Desde el borde superior
                for (int x = 0; x < size; x++) {
                    int distancia = dijkstra.getDistance(new Point(size - 1, y)); // Hasta el borde inferior
                    if (distancia != Integer.MAX_VALUE) {
                        score += 1000 - distancia; // Premia caminos cortos
>>>>>>> c3fd4fc (Tfdjssh)
                    }
                }
            }
        }

        return bloqueos;
    }

    private int[][] construirTablero(HexGameStatus gameState, int size) {
        int[][] board = new int[size][size];
        for (int x = 0; x < size; x++) {
            for (int y = 0; y < size; y++) {
                board[x][y] = gameState.getPos(x, y);
            }
        }
        return board;
    }

    private ArrayList<Point> obtenerFichasPropias(HexGameStatus gameState, int playerId) {
        ArrayList<Point> fichas = new ArrayList<>();
        int size = gameState.getSize();
        for (int x = 0; x < size; x++) {
            for (int y = 0; y < size; y++) {
                if (gameState.getPos(x, y) == playerId) {
                    fichas.add(new Point(x, y));
                }
            }
        }
        return fichas;
    }

    private int evaluarPosicionesEstrategicas(HexGameStatus gameState, int myId, int rivalId, Point ficha, int size) {
        int score = 0;
        Point[] diagonales = {
            new Point(2, -1), new Point(1, 1), new Point(-1, 2),
            new Point(-2, 1), new Point(-1, -1), new Point(1, -2)
        };

        for (Point dir : diagonales) {
            Point vecino = new Point(ficha.x + dir.x, ficha.y + dir.y);
            if (isInsideBoard(vecino, size)) {
                int cell = gameState.getPos(vecino.x, vecino.y);
                if (cell == myId) {
                    score += 50;
                    Point inter1 = new Point(ficha.x + dir.x / 2, ficha.y + dir.y / 2);
                    Point inter2 = new Point(ficha.x + dir.x / 2 + dir.y / 2, ficha.y + dir.y / 2 - dir.x / 2);

                    if (isInsideBoard(inter1, size) && isInsideBoard(inter2, size)) {
                        int cell1 = gameState.getPos(inter1.x, inter1.y);
                        int cell2 = gameState.getPos(inter2.x, inter2.y);

                        if ((cell1 == 0 && cell2 == 0) || (cell1 == myId && cell2 == 0) || (cell1 == 0 && cell2 == myId)) {
                            score += 100;
                        } else if (cell1 == rivalId && cell2 == rivalId) {
                            score -= 50;
                        } else if (cell1 == myId && cell2 == myId) {
                            score += 200;
                        }
                    }
                } else if (cell == rivalId) {
                    score -= 20;
                }
            }
        }
        return score;
    }

    private int calcularDistanciaBordes(HexGameStatus gameState, int playerId, Point ficha, boolean vertical) {
        int size = gameState.getSize();
        boolean[][] visitadas = new boolean[size][size];
        ArrayList<Point> cola = new ArrayList<>();
        cola.add(ficha);
        visitadas[ficha.x][ficha.y] = true;

        int distancia = 0;
        while (!cola.isEmpty()) {
            ArrayList<Point> siguienteNivel = new ArrayList<>();

            for (Point actual : cola) {
                if (haLlegadoABorde(actual, vertical, size)) {
                    return distancia;
                }

                for (Point vecino : gameState.getNeigh(actual)) {
                    if (vecino != null && isInsideBoard(vecino, size) && !visitadas[vecino.x][vecino.y] && gameState.getPos(vecino.x, vecino.y) != -playerId) {
                        siguienteNivel.add(vecino);
                        visitadas[vecino.x][vecino.y] = true;
                    }
                }
            }

            cola = siguienteNivel;
            distancia++;
        }

        return Integer.MAX_VALUE;
    }

    private boolean haLlegadoABorde(Point punto, boolean vertical, int size) {
        return vertical ? (punto.x == 0 || punto.x == size - 1) : (punto.y == 0 || punto.y == size - 1);
    }

    private int bloquearRival(HexGameStatus gameState, int rivalId, int size) {
        int bloqueos = 0;
        Point[] direcciones = {
            new Point(1, 0), new Point(0, 1), new Point(1, -1),
            new Point(-1, 0), new Point(0, -1), new Point(-1, 1)
        };

        for (int x = 0; x < size; x++) {
            for (int y = 0; y < size; y++) {
                if (gameState.getPos(x, y) == rivalId) {
                    for (Point dir : direcciones) {
                        Point vecino = new Point(x + dir.x, y + dir.y);
                        if (isInsideBoard(vecino, size) && gameState.getPos(vecino.x, vecino.y) == 0) {
                            bloqueos += 500; // Penaliza espacios críticos alrededor del rival
                        }
                    }
                }
            }
        }

        return bloqueos;
    }

    private boolean isInsideBoard(Point p, int size) {
        return p.x >= 0 && p.x < size && p.y >= 0 && p.y < size;
    }
}
