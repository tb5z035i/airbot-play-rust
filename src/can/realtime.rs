use libc::{
    MCL_CURRENT, MCL_FUTURE, SCHED_FIFO, cpu_set_t, mlockall, pthread_self, pthread_setaffinity_np,
    sched_param, sched_setscheduler,
};
use std::io;
use std::mem;

pub fn configure_sched_fifo(priority: i32) -> io::Result<()> {
    let mut params: sched_param = unsafe { mem::zeroed() };
    params.sched_priority = priority;
    let result = unsafe { sched_setscheduler(0, SCHED_FIFO, &params) };
    if result == 0 {
        Ok(())
    } else {
        Err(io::Error::last_os_error())
    }
}

pub fn lock_memory() -> io::Result<()> {
    let result = unsafe { mlockall(MCL_CURRENT | MCL_FUTURE) };
    if result == 0 {
        Ok(())
    } else {
        Err(io::Error::last_os_error())
    }
}

pub fn set_current_thread_affinity(core_id: usize) -> io::Result<()> {
    let mut cpu_set: cpu_set_t = unsafe { mem::zeroed() };
    unsafe {
        libc::CPU_ZERO(&mut cpu_set);
        libc::CPU_SET(core_id, &mut cpu_set);
    }

    let result = unsafe { pthread_setaffinity_np(pthread_self(), mem::size_of::<cpu_set_t>(), &cpu_set) };
    if result == 0 {
        Ok(())
    } else {
        Err(io::Error::from_raw_os_error(result))
    }
}
