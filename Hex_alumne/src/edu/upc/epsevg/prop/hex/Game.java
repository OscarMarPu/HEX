package edu.upc.epsevg.prop.hex;

import edu.upc.epsevg.prop.hex.players.H_E_X_Player;
import edu.upc.epsevg.prop.hex.players.PlayerMinimax;



import javax.swing.SwingUtilities;

/**
 * Checkers: el joc de taula.
 * @author bernat
 */
public class Game {
        /**
     * @param args
     */
    public static void main(String[] args) { 
        
        SwingUtilities.invokeLater(new Runnable() {
            @Override
            public void run() {
                
                IPlayer player1 = new H_E_X_Player(2/*GB*/);
                
                IPlayer player2 = new PlayerMinimax("OscarCabezas", 4);
                                
                new Board(player1 , player2, 11 /*mida*/,  10/*s*/, false);
             }
        });
    }
}