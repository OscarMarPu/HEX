package edu.upc.epsevg.prop.hex.players;

import edu.upc.epsevg.prop.hex.HexGameStatus;
import edu.upc.epsevg.prop.hex.PlayerType;

public class HexHeuristica {

    // Vector de vecinos para hex (6 direcciones)
    private static final int[][] DIRECTIONS = {
        {-1,0}, {1,0}, {0,-1}, {0,1}, {-1,1}, {1,-1}
    };

    public static int evaluate(HexGameStatus s, PlayerType myColor) {
        // Si el juego ha terminado, valor extremo
        if (s.isGameOver()) {
            PlayerType winner = s.GetWinner();
            if (winner == myColor) {
                return Integer.MAX_VALUE - 1;
            } else {
                return Integer.MIN_VALUE + 1;
            }
        }

        PlayerType opponentColor = (myColor == PlayerType.PLAYER1) ? PlayerType.PLAYER2 : PlayerType.PLAYER1;
        int myPieceVal = (myColor == PlayerType.PLAYER1) ? 1 : -1;
        int enemyPieceVal = (myColor == PlayerType.PLAYER1) ? -1 : 1;

        // Calculamos distancias
        int myDistance = Dijkstra.getShortestPath(s, myColor);
        int oppDistance = Dijkstra.getShortestPath(s, opponentColor);

        // Dar mayor peso a la diferencia de distancias
        // Antes: factor 10, ahora factor 50 para que prime conectar lados
        int value = (oppDistance - myDistance) * 50;

        int size = s.getSize();
        double center = (size - 1) / 2.0;
        int centerScore = 0;

        // Reducimos impacto del centro: multiplicamos por 1 en vez de 2
        double centerFactor = 1.0;
        
        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {
                int cell = s.getPos(i, j);
                if (cell == myPieceVal) {
                    double dx = i - center;
                    double dy = j - center;
                    double distCenter = Math.sqrt(dx*dx + dy*dy);
                    double maxDistCenter = size/2.0;
                    int cScore = (int)((maxDistCenter - distCenter)*centerFactor);
                    centerScore += cScore;
                }
            }
        }

        value += centerScore;

        // Reducimos valor del doble bloqueo a 10
        int doubleBlockScore = 0;
        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {
                int cell = s.getPos(i, j);
                if (cell == 0) {
                    int blockedExits = countBlockedEnemyExits(s, i, j, enemyPieceVal);
                    if (blockedExits >= 2) {
                        doubleBlockScore += 10; 
                    }
                }
            }
        }

        value += doubleBlockScore;

        return value;
    }

    private static int countBlockedEnemyExits(HexGameStatus s, int x, int y, int enemyVal) {
        int size = s.getSize();
        int enemyCount = 0;
        for (int[] d : DIRECTIONS) {
            int nx = x + d[0];
            int ny = y + d[1];
            if (nx >= 0 && nx < size && ny >= 0 && ny < size) {
                int cell = s.getPos(nx, ny);
                if (cell == enemyVal) {
                    enemyCount++;
                }
            }
        }
        return enemyCount;
    }
}
