/*
 * Sistema de threads em nivel de usuario — miniprojeto SO
 *
 * Compilacao (Linux / WSL / macOS com suporte a ucontext):
 *   gcc -std=c11 -Wall -Wextra -O0 -g main.c -o uthreads
 *
 * Nota: ucontext esta obsoleto no POSIX moderno, mas permanece amplamente
 * disponivel em glibc para fins didaticos. No Windows nativo, use WSL ou
 * uma VM Linux para compilar e executar.
 */

#define _GNU_SOURCE

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <ucontext.h>

/* ------------------------------------------------------------------------- */
/* Constantes e tipos                                                         */
/* ------------------------------------------------------------------------- */

#define NUM_THREADS 5
#define STACK_SIZE 16384
#define INCREMENTS_PER_THREAD 50000000u

/** Estados da thread no escalonador simplificado. */
typedef enum {
    THREAD_ST_READY = 0,
    THREAD_ST_RUNNING,
    THREAD_ST_TERMINATED
} thread_state_t;

/**
 * Representa uma thread de usuario: contexto da CPU, pilha dedicada,
 * identificador e estado logico para o escalonador.
 */
typedef struct user_thread {
    ucontext_t context; /* registradores + PC + ponteiro de pilha do kernel */
    _Alignas(16) char stack[STACK_SIZE]; /* alinhamento util para algumas ABIs */
    int id;
    thread_state_t state;
} user_thread_t;

/* Pool fixo de 5 threads conforme enunciado. */
static user_thread_t g_threads[NUM_THREADS];

/** Contexto do thread principal — destino para onde o dispatcher retorna. */
static ucontext_t g_main_uctx;

/** Indice da thread atualmente em execucao (0 .. NUM_THREADS-1). */
static int g_current_tid = -1;

/**
 * Quando nao-zero, a regiao critica que altera o contador compartilhado
 * protege contra preempcao por SIGALRM (mascara de sinais).
 * Alternamos isso entre fases da main para ilustrar corrida vs correcao.
 */
static volatile sig_atomic_t g_protect_critical_section = 0;

/** Contador global compartilhado — alvo da secao critica. */
static volatile unsigned long g_shared_counter = 0;

static volatile sig_atomic_t g_alarm_need_reschedule = 0;

/* Prototipos */
static int scheduler_round_robin(int from_tid);
static void dispatcher_switch(int from_tid, int to_tid);
static void dispatcher_to_main(int from_tid);
static void process_preemption_if_needed(void);
static void thread_trampoline(unsigned int tid);
static void yield_from_thread(int tid);
static void init_thread_contexts(void);
static void run_thread_system(void);
static void setup_alarm_handler(void);
static void teardown_alarm_handler(void);

/* ------------------------------------------------------------------------- */
/* Escalonador (Round-Robin circular)                                         */
/* ------------------------------------------------------------------------- */

/**
 * Escolhe a proxima thread a executar, ignorando as que ja terminaram.
 * Politica Round-Robin: a partir de (from_tid + 1), percorre o anel
 * 0..NUM_THREADS-1. Com from_tid < 0 (bootstrap), retorna a primeira
 * thread ainda nao terminada.
 */
static int scheduler_round_robin(int from_tid)
{
    if (from_tid < 0) {
        for (int i = 0; i < NUM_THREADS; ++i) {
            if (g_threads[i].state != THREAD_ST_TERMINATED) {
                return i;
            }
        }
        return -1;
    }

    for (int step = 1; step <= NUM_THREADS; ++step) {
        int i = (from_tid + step) % NUM_THREADS;
        if (g_threads[i].state != THREAD_ST_TERMINATED) {
            return i;
        }
    }
    return -1;
}

/* ------------------------------------------------------------------------- */
/* Dispatcher — troca real de contexto                                        */
/* ------------------------------------------------------------------------- */

/**
 * Realiza a troca de contexto entre duas threads validas usando swapcontext().
 * Atualiza g_current_tid e estados READY/RUNNING de forma consistente.
 */
static void dispatcher_switch(int from_tid, int to_tid)
{
    if (from_tid < 0 || from_tid >= NUM_THREADS || to_tid < 0 ||
        to_tid >= NUM_THREADS || from_tid == to_tid) {
        return;
    }

    printf("--- troca: Thread %d -> Thread %d ---\n", from_tid, to_tid);
    fflush(stdout);

    /*
     * Thread que ja terminou permanece TERMINATED; as demais voltam a READY
     * quando sao preemptadas.
     */
    if (g_threads[from_tid].state == THREAD_ST_RUNNING) {
        g_threads[from_tid].state = THREAD_ST_READY;
    }
    g_current_tid = to_tid;
    g_threads[to_tid].state = THREAD_ST_RUNNING;

    /* A magica do SO em nivel de usuario: CPU passa a executar outra pilha. */
    if (swapcontext(&g_threads[from_tid].context, &g_threads[to_tid].context) !=
        0) {
        perror("swapcontext");
        _exit(EXIT_FAILURE);
    }
}

/** Retorna do dispatcher para o contexto main (fim de um lote de threads). */
static void dispatcher_to_main(int from_tid)
{
    if (from_tid < 0 || from_tid >= NUM_THREADS) {
        return;
    }
    g_current_tid = -1;
    if (swapcontext(&g_threads[from_tid].context, &g_main_uctx) != 0) {
        perror("swapcontext(main)");
        _exit(EXIT_FAILURE);
    }
}

/* ------------------------------------------------------------------------- */
/* Tratamento de preempcao (quantum simulado)                                 */
/* ------------------------------------------------------------------------- */

static void alarm_handler(int signo)
{
    (void)signo;

    /*
     * Apenas sinalizar; trabalho pesado (escalonar + swapcontext) fora do
     * handler evita reentrancia dupla e mantem o handler minimo. O fluxo
     * volta ao trampoline/worker que chama process_preemption_if_needed().
     */
    g_alarm_need_reschedule = 1;

    /* rearma o quantum de ~1 segundo */
    alarm(1);
}

static void setup_alarm_handler(void)
{
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = alarm_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART;

    if (sigaction(SIGALRM, &sa, NULL) != 0) {
        perror("sigaction(SIGALRM)");
        exit(EXIT_FAILURE);
    }
}

static void teardown_alarm_handler(void)
{
    alarm(0);
    signal(SIGALRM, SIG_DFL);
}

/**
 * Chamado com frequencia dentro das threads: se o alarme pediu, aplicamos
 * RR + dispatcher aqui (fora do async signal handler).
 */
static void process_preemption_if_needed(void)
{
    if (!g_alarm_need_reschedule) {
        return;
    }
    g_alarm_need_reschedule = 0;

    int from = g_current_tid;
    if (from < 0) {
        return;
    }

    int to = scheduler_round_robin(from);
    if (to < 0 || to == from) {
        return;
    }

    dispatcher_switch(from, to);
}

/* ------------------------------------------------------------------------- */
/* Logica de cada thread de usuario                                           */
/* ------------------------------------------------------------------------- */

/**
 * Corpo logico da thread: muitos incrementos ao contador global.
 * Sem protecao: carga/incremento/gravacao com pontos de preempcao no meio
 * expoem lost update (condicao de corrida).
 * Com protecao: SIGALRM bloqueado na secao critica — seguro em relacao a
 * esse quantum (atende o enunciado; nao substitui um mutex multi-processo).
 */
static void thread_work_body(int tid)
{
    (void)tid;

    for (unsigned i = 0; i < INCREMENTS_PER_THREAD; ++i) {
        process_preemption_if_needed();

        if (g_protect_critical_section) {
            sigset_t block, old;
            sigemptyset(&block);
            sigaddset(&block, SIGALRM);

            if (sigprocmask(SIG_BLOCK, &block, &old) != 0) {
                perror("sigprocmask(block)");
                _exit(EXIT_FAILURE);
            }

            /* Secao critica: sem entrega de SIGALRM => sem quantum aqui. */
            g_shared_counter++;

            if (sigprocmask(SIG_SETMASK, &old, NULL) != 0) {
                perror("sigprocmask(restore)");
                _exit(EXIT_FAILURE);
            }
        } else {
            /*
             * Secao critica IMPLICITA sem sincronismo: dividimos o ++ em passos
             * e checamos preempcao no meio. Outra thread pode avancar o
             * global; ao gravar um 'v' desatualizado perdem-se atualizacoes
             * (lost update).
             */
            unsigned long v = g_shared_counter;
            process_preemption_if_needed();
            v++;
            process_preemption_if_needed();
            g_shared_counter = v;
        }

        /* permite interrupcoes entre iteracoes para misturar threads */
        if ((i & 1023u) == 0u) {
            process_preemption_if_needed();
        }
    }
}

/**
 * makecontext exige void (*)(void); passamos o tid como unsigned e
 * convertemos no trampolim (padrao portatil em 32/64 bits com um unico int).
 */
static void thread_trampoline(unsigned int tid_u)
{
    const int tid = (int)tid_u;

    if (tid < 0 || tid >= NUM_THREADS) {
        _exit(EXIT_FAILURE);
    }

    printf("[Thread %d] iniciando execucao\n", tid);
    fflush(stdout);

    g_threads[tid].state = THREAD_ST_RUNNING;
    thread_work_body(tid);

    g_threads[tid].state = THREAD_ST_TERMINATED;
    printf("[Thread %d] finalizada\n", tid);
    fflush(stdout);
    yield_from_thread(tid);
}

/** Apos terminar o trabalho, devolve a CPU a outra thread ou ao main. */
static void yield_from_thread(int tid)
{
    int next = scheduler_round_robin(tid);
    if (next < 0) {
        dispatcher_to_main(tid);
        return;
    }
    dispatcher_switch(tid, next);
}

/* ------------------------------------------------------------------------- */
/* Inicializacao dos contextos (uma vez por fase experimental)               */
/* ------------------------------------------------------------------------- */

static void init_thread_contexts(void)
{
    for (int i = 0; i < NUM_THREADS; ++i) {
        memset(&g_threads[i], 0, sizeof(g_threads[i]));
        g_threads[i].id = i;
        g_threads[i].state = THREAD_ST_READY;

        if (getcontext(&g_threads[i].context) != 0) {
            perror("getcontext(thread)");
            exit(EXIT_FAILURE);
        }

        g_threads[i].context.uc_stack.ss_sp = g_threads[i].stack;
        g_threads[i].context.uc_stack.ss_size = sizeof(g_threads[i].stack);
        g_threads[i].context.uc_link = &g_main_uctx;

        /* int-arity: um unico argumento identifica a thread */
        makecontext(&g_threads[i].context, (void (*)(void))thread_trampoline, 1,
                    (int)(unsigned)i);
    }
}

/**
 * Captura o contexto principal e dispara a primeira thread.
 * Quando todas terminam, alguma cadeia de swapcontext volta aqui.
 */
static void run_thread_system(void)
{
    static volatile int s_returned = 0;
    s_returned = 0;

    if (getcontext(&g_main_uctx) != 0) {
        perror("getcontext(main)");
        exit(EXIT_FAILURE);
    }

    /* Guarda contra re-entrada: quando o contexto principal e retomado
     * via uc_link ou dispatcher_to_main, s_returned ja estara em 1 e
     * retornamos imediatamente sem iniciar um novo ciclo. */
    if (s_returned) {
        return;
    }
    s_returned = 1;

    g_current_tid = -1;

    int first = scheduler_round_robin(-1);
    if (first < 0) {
        fprintf(stderr, "Nenhuma thread inicializavel.\n");
        exit(EXIT_FAILURE);
    }

    g_current_tid = first;
    g_threads[first].state = THREAD_ST_RUNNING;

    alarm(1);

    if (swapcontext(&g_main_uctx, &g_threads[first].context) != 0) {
        perror("swapcontext(start)");
        exit(EXIT_FAILURE);
    }

    alarm(0);
}

/* ------------------------------------------------------------------------- */
/* main — duas fases: corrida livre vs secao critica protegida               */
/* ------------------------------------------------------------------------- */

int main(void)
{
    const unsigned long expected =
        (unsigned long)NUM_THREADS * (unsigned long)INCREMENTS_PER_THREAD;

    printf("Mini-projeto: threads em nivel de usuario (ucontext + SIGALRM)\n");
    printf("%d threads, %u incrementos cada, quantum ~1 s (alarm)\n\n",
           NUM_THREADS, INCREMENTS_PER_THREAD);

    setup_alarm_handler();

    /* --- Fase 1: sem protecao — espera-se valor menor que o teorico --- */
    g_protect_critical_section = 0;
    g_shared_counter = 0;
    init_thread_contexts();

    printf("=== Fase 1: contador compartilhado SEM protecao ===\n");
    printf("(preempcao pode ocorrer entre leitura, +1 e escrita do contador)\n\n");

    run_thread_system();
    teardown_alarm_handler();

    printf("Valor esperado (sem perdas): %lu\n", expected);
    printf("Valor obtido:                 %lu\n", g_shared_counter);
    if (g_shared_counter != expected) {
        printf(
            "=> Condicao de corrida: resultado inconsistente com o esperado.\n\n");
    } else {
        printf("=> Nenhuma divergencia nesta execucao (pode variar entre runs).\n\n");
    }

    /* --- Fase 2: mascara SIGALRM na secao critica --- */
    setup_alarm_handler();
    g_protect_critical_section = 1;
    g_shared_counter = 0;
    init_thread_contexts();

    printf("=== Fase 2: MESMA carga COM bloqueio de SIGALRM na secao critica "
           "===\n\n");

    run_thread_system();
    teardown_alarm_handler();

    printf("Valor esperado: %lu\n", expected);
    printf("Valor obtido:   %lu\n", g_shared_counter);
    if (g_shared_counter == expected) {
        printf("=> Comportamento correto: todos os incrementos foram "
               "preservados.\n");
    } else {
        printf("=> Ainda houve divergencia — verifique ambiente / sinal.\n");
    }

    return EXIT_SUCCESS;
}