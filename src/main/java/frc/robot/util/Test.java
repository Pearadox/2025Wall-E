// package frc.robot.util;

// import edu.wpi.first.util.datalog.DataLogIterator;
// import edu.wpi.first.util.datalog.DataLogReader;
// import edu.wpi.first.util.datalog.DataLogRecord;
// import edu.wpi.first.util.datalog.DataLogRecord.StartRecordData;

// import java.util.*;

// public class Test {
//     try {
//         DataLogReader dr = new DataLogReader("example_log.wpilog");
//         DataLogIterator dri = dr.iterator();
//         DataLogRecord rcd = dri.next();
//         StartRecordData srd = rcd.getStartData();
//         long time = rcd.getTimestamp();
//         String s = rcd.getString();
//         String[] ss = rcd.getStringArray();
//         long n = rcd.getInteger();
//         if (dri.hasNext()) {
//             dri.next();
//         }
//     } catch (Exception e) {
//         e.printStackTrace();
//     }
// }
